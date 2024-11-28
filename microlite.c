#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/audio.h>
#include <sound/rawmidi.h>
#include <linux/usb/midi.h>
#include <linux/usb/quirks.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/asequencer.h>

// The micro lite offers modes for using either two Interrupt endpoints, two Bulk
// endpoints, or an Isochronous output endpoint and an Interrupt input endpoint.
// The Bulk endpoints should work to an extent, but hasn't been vigorously tested.
// The isochronous endpoint hasn't been reverse-engineered at all.
#define PIPE_TYPE PIPE_INTERRUPT
#define TIMING 1
#define MIDITIME_NSEC 320000

MODULE_DESCRIPTION("MOTU micro lite driver");
MODULE_AUTHOR("Ashley Miller <ashley.pdtn85@gmail.com>");
MODULE_LICENSE("MIT");
MODULE_SOFTDEP("pre: snd_rawmidi");

#define OUTPUT_URBS 4
#define INPUT_URBS  4
#define ERROR_DELAY_JIFFIES (HZ / 10)

static int          index[SNDRV_CARDS]  = SNDRV_DEFAULT_IDX;
static char*        id[SNDRV_CARDS]     = SNDRV_DEFAULT_STR;
static bool         enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;

static struct usb_driver motu_driver;

struct Microlite_t;
struct MicroliteEndpoint_t;
struct MicroliteEndpointOut_t;
struct MicroliteEndpointIn_t;

static int  MidiOutput_Open(struct snd_rawmidi_substream *substream);
static int  MidiOutput_Close(struct snd_rawmidi_substream *substream);
static void MidiOutput_Trigger(struct snd_rawmidi_substream *substream, int up);
static void MidiOutput_Drain(struct snd_rawmidi_substream *substream);
static int  MidiInput_Open(struct snd_rawmidi_substream *substream);
static int  MidiInput_Close(struct snd_rawmidi_substream *substream);
static void MidiInput_Trigger(struct snd_rawmidi_substream *substream, int up);
static void Microlite_GetPortInfo(struct snd_rawmidi *rmidi, int number, struct snd_seq_port_info *seq_port_info);
static void MicroliteRawMidiFree(struct snd_rawmidi* rawmidi);
static void OnOutUrbCompleted(struct urb *urb);
static void OnInUrbCompleted(struct urb *urb); // snd_usbmidi_in_urb_complete
static int  CreateOutEndpoint(struct Microlite_t *microlite, int endpointNum);
static int  CreateInEndpoint(struct Microlite_t *microlite, int endpointId /* , int endpointInterval */ );
static void CreateSubstream(struct Microlite_t *umidi, int stream, int number, struct snd_rawmidi_substream **rsubstream);
static void FreeUrb(struct Microlite_t *microlite, struct urb *urb, unsigned int buffer_length); // free_urb_and_buffer
static void DeleteInEndpoint(struct MicroliteEndpointIn_t *endpoint); // snd_usbmidi_in_endpoint_delete
static void DeleteOutEndpoint(struct MicroliteEndpointOut_t *endpoint); // snd_usbmidi_out_endpoint_delete
static void ClearOutEndpoint(struct MicroliteEndpointOut_t *endpoint); // snd_usbmidi_out_endpoint_clear
static int  SubmitUrb(struct urb *urb, gfp_t flags); // snd_usbmidi_submit_urb
static int  UrbError(const struct urb *urb); // snd_usbmidi_urb_error
static void MicroliteDisconnect(/*struct list_head *p*/ struct Microlite_t* microlite); // snd_usbmidi_disconnect
static void DumpUrb(const char *type, const u8 *data, int length);
static void MicroliteErrorTimer(struct timer_list *t); // snd_usbmidi_error_timer
static void OnPortTransmissionFinished(struct timer_list *);

static const struct snd_rawmidi_ops s_MotuMidiOutOps;
static const struct snd_rawmidi_ops s_MotuMidiInOps;

static const struct snd_rawmidi_ops s_MotuMidiOutOps = {
	.open    = MidiOutput_Open,
	.close   = MidiOutput_Close,
	.trigger = MidiOutput_Trigger,
	.drain   = MidiOutput_Drain,
};

static const struct snd_rawmidi_ops s_MotuMidiInOps = {
	.open    = MidiInput_Open,
	.close   = MidiInput_Close,
	.trigger = MidiInput_Trigger
};
static const struct snd_rawmidi_global_ops s_MotuMidiGlobalOps = {
	.get_port_info = Microlite_GetPortInfo,
};

struct Microlite_t
{
	struct usb_device   * Device;
	struct snd_card     * Card;
	struct usb_interface* Interface;
	int                   CardIndex;
//	struct list_head      DeviceList; 
	struct timer_list     ErrorTimer;
	spinlock_t            BufferLock;
	spinlock_t            Lock;
	struct mutex          Mutex;
	unsigned long         State;
	struct snd_rawmidi  * RawMidi;
	//struct work_struct    MidiOutWork;
	spinlock_t            DisconnectLock; // disc_lock
	struct rw_semaphore   DisconnectSemaphore; // disc_rwsem
	unsigned long         InputTriggered; // bit mask for which input ports have been triggered
	unsigned char         Disconnected;
	unsigned char         InputRunning;
	struct mutex          SubstreamStateMutex; // mutex
	unsigned int          SubstreamOpenCount[2]; // opened
	ktime_t               StartTime;
	struct MicroliteEndpoint_t
	{
		struct MicroliteEndpointOut_t *Out;
		struct MicroliteEndpointIn_t  *In;
	} Endpoints;
};

struct MicroliteEndpointOut_t
{
	struct Microlite_t *Microlite;
	struct OutUrbContext_t
	{
		struct urb *Urb;
		// Time in monotonic nanoseconds when each port is expected to have finished its transmission.
		u64 CompletionTimes[5];
		// Bitmask to indicate which ports had data on them, and hence which array items in `CompletionTimes` are valid.
		u8 Ports;
		struct MicroliteEndpointOut_t *Endpoint;
	} Urbs[OUTPUT_URBS];
	unsigned int       ActiveUrbs;
	unsigned int       DrainUrbs;
	int                MaxTransfer;		/* size of urb buffer */
	struct work_struct Work;
	unsigned int       NextUrb;
	spinlock_t         BufferLock;
	uint8_t            PortsWaiting;

	struct MicroliteOutPort_t
	{
		struct MicroliteEndpointOut_t *Endpoint;
		struct snd_rawmidi_substream *Substream;
#if defined(TIMING) && TIMING
		ktime_t TimeWhenTransferWillFinish; // Expected timestamp when the MIDI data will be finished transferring at the standard MIDI baud.
		struct timer_list Timer;
		//ktime_t TimeOfLastTransfer;
		//uint16_t QueueLengthAtTimeOfLastTransfer;
#endif
		uint8_t ActiveUrbs;
		uint8_t Cable;
		int     Active;
		int     Stall;
		uint8_t Data[2];
	} Ports[5];
	int CurrentPort;

	wait_queue_head_t DrainWait;
	wait_queue_head_t OverflowWait;
};

struct MicroliteEndpointIn_t
{
	struct Microlite_t *Microlite;
	struct urb *Urbs[INPUT_URBS];
	struct MicroliteInPort_t
	{
		struct snd_rawmidi_substream *Substream;
		u8 RunningStatusLength;
	} Ports[5];
	u8 seen_f5;
	bool InSysex;
	u8 LastCin;
	u8 ErrorResubmit;
	int CurrentPort;
};

static DEFINE_MUTEX(s_devicesMutex);
static unsigned int s_devicesUsed;

static void FreeUsbResources(struct Microlite_t* microlite, struct usb_interface* interface);

static void motu_card_free(struct snd_card *card)
{
	struct Microlite_t* ua = card->private_data;
	mutex_destroy(&ua->Mutex);
}

static void MidiDoOutput(struct MicroliteEndpointOut_t *endpoint);
static void MicroliteMidiOutWork(struct work_struct *work)
{
	struct MicroliteEndpointOut_t *endpoint = container_of(work, struct MicroliteEndpointOut_t, Work);

	//printk(KERN_INFO "MicroliteMidiOutWork\n");
	MidiDoOutput(endpoint);
};

static void PrepareMidiByteForUsb(struct MicroliteOutPort_t *port, uint8_t b, struct urb *urb);

/*
 * This is called when some data should be transferred to the device
 * (from one or more substreams).
 */
static void MidiDoOutput(struct MicroliteEndpointOut_t *endpoint)
{
	unsigned int urb_index, flags;
	struct urb *urb;
	struct MicroliteOutPort_t *port;
	int p, err;
	uint8_t b;
	uint8_t *pHead, *pData;
	int nBytesInThisBlock;
	const uint8_t *pEnd;
#if defined(TIMING) && TIMING
	u64 timeNow;
	union
	{
		u8 b[5];
		u64 u64;
	} bytesPerPort;
	uint8_t portsInUse;
#endif

	//printk(KERN_INFO "MidiDoOutput\n");

	guard(spinlock_irqsave)(&endpoint->BufferLock);
	//spin_lock_irqsave(&endpoint->BufferLock, flags);
	if (endpoint->Microlite->Disconnected)
	{
		//spin_unlock_irqrestore(&endpoint->BufferLock, flags);
		return;
	}

	// In my loopback test, I receive back 1314 bytes out of 5574. I assume there's a 1KB buffer
	// locally on the MicroLite (and there's some overlap).
	// To prevent overruns, I should
	//  1) Keep track of timestamps when a URB was sent, and the number of MIDI bytes.
	//  2) Calculate the difference in time, and work out how many of those MIDI bytes have successfully sent.
	//  3) Instead of keeping track on each URB, just keep a running total at the time of the last transfer.
	//  4) If the calculated queue length exceeds 1024 bytes, stall.
	// This only requires simple calculations to be performed at dispatch time, instead of running timers, etc.
	// I don't yet know if this 1KB buffer in the MicroLite is global or per-port. Let's just assume per-port for now.
#if defined(TIMING) && TIMING
	timeNow = ktime_get_ns();
	bytesPerPort.u64 = 0;
#endif

	urb_index = endpoint->NextUrb;
	for (;;)
	{
		portsInUse = 0;
		if (!(endpoint->ActiveUrbs & (1 << urb_index)))
		{
			urb = endpoint->Urbs[urb_index].Urb;
			urb->transfer_buffer_length = 0;

			pHead  = &((uint8_t*)urb->transfer_buffer)[2];
			pData  = &((uint8_t*)urb->transfer_buffer)[3];
			pEnd   = &((uint8_t*)urb->transfer_buffer)[endpoint->MaxTransfer];

		NextBlock:
			nBytesInThisBlock = 0;
			for (p = 0; p < 5; ++p)
			{
				port = &endpoint->Ports[p];
				if (!port->Active)
					continue;

				*pHead = 0;
			
				// get TX data for rawmidi stream
				if (snd_rawmidi_transmit(port->Substream, &b, 1) == 1)
				{
#if defined(TIMING) && TIMING
					bytesPerPort.b[p]++;
#endif
					if (nBytesInThisBlock++ == 0)
						++urb->transfer_buffer_length; // because now we're counting the head byte as well
					portsInUse |= (1 << p);
					*pHead |= (1 << p);
					*pData++ = b;
					urb->transfer_buffer_length++;
					if (pData == pEnd)
						goto EndUrb;
				}
			}

			if (*pHead)
			{
				pHead = pData;
				pData = pHead + 1;
				if (pData < pEnd)
					goto NextBlock;
			}
		
		EndUrb:
			if (urb->transfer_buffer_length == 0)
				break;
			urb->transfer_buffer_length += 2; // to account for the timestamp bytes, or whatever they are

			//DumpUrb("sending", urb->transfer_buffer, urb->transfer_buffer_length);
			urb->dev = endpoint->Microlite->Device;
			if (SubmitUrb(urb, GFP_ATOMIC) < 0)
			{
				dev_warn(&endpoint->Microlite->Device->dev, "Failed submitting URB\n");
				break;
			}
			endpoint->PortsWaiting |= portsInUse;
			endpoint->ActiveUrbs |= 1 << urb_index;
		}

		if (++urb_index >= OUTPUT_URBS)
			urb_index = 0;
		if (urb_index == endpoint->NextUrb)
			break;
	}
	endpoint->NextUrb = urb_index;

#if defined(TIMING) && TIMING
	//timeNow = ktime_get();
	for (p = 0; p < 5; ++p)
	{
		if (bytesPerPort.b[p] == 0)
			continue;

		u64 portTime = port->TimeWhenTransferWillFinish;
		if (timeNow > portTime)
			portTime = timeNow;
		portTime += bytesPerPort.b[p] * MIDITIME_NSEC;
		port->TimeWhenTransferWillFinish = portTime;

		mod_timer(&port->Timer, usecs_to_jiffies(ktime_sub_ns(portTime, timeNow) / 1000));
	}
#endif
	//spin_unlock_irqrestore(&endpoint->BufferLock, flags);
};


static int motu_probe_handler(struct usb_interface *interface, const struct usb_device_id *usb_id)
{
	int                               err, i, j;
	unsigned int                      cardIndex;
	struct snd_card                 * card;
	struct Microlite_t              * microlite;
	struct usb_host_interface       * hostInterface;
	struct usb_interface_descriptor * interfaceDescriptor;
	struct usb_endpoint_descriptor  * endpointDescriptor;
	const struct usb_device_descriptor * deviceDescriptor;

	//printk(KERN_INFO "MOTU Probe(...)\n");

	deviceDescriptor = &__intf_to_usbdev_const(interface)->descriptor;
	// Using `usb_id` to find out information about the device doesn't work, as `usb_id` only contains
	// the information we provided in this module's ID table `motu_ids`.
	if (deviceDescriptor->bDeviceClass != 0xFF || deviceDescriptor->bDeviceSubClass != 3)
	{
		printk(KERN_ERR "Invalid device class or subclass %02Xh:%02Xh\n", usb_id->bDeviceClass, usb_id->bDeviceSubClass);
		return -ENODEV;
	}
	if (strcmp(__intf_to_usbdev_const(interface)->product, "micro lite") != 0)
	{
		printk(KERN_ERR "Mismatch of product text: '%s'\n", __intf_to_usbdev_const(interface)->product);
		return -ENODEV;
	}

	guard(mutex)(&s_devicesMutex);
	//mutex_lock(&s_devicesMutex);

	for (cardIndex = 0; cardIndex < SNDRV_CARDS; ++cardIndex)
		if (enable[cardIndex] && !(s_devicesUsed & (1 << cardIndex)))
			break;
	if (cardIndex >= SNDRV_CARDS)
	{
		//mutex_unlock(&s_devicesMutex);
		printk(KERN_ERR "No card slots available\n");
		return -ENOENT;
	}
	//printk(KERN_INFO "Creating new sound card device\n");
	err = snd_card_new(&interface->dev,
	                   index[cardIndex],
	                   id[cardIndex],
	                   THIS_MODULE,
	                   sizeof(*microlite),
	                   &card);
	if (err < 0)
	{
		//mutex_unlock(&s_devicesMutex);
		return err;
	}

	card->private_free    = motu_card_free;
	microlite             = card->private_data;
	microlite->Device     = interface_to_usbdev(interface);
	microlite->Card       = card;
	microlite->CardIndex  = cardIndex;
	microlite->StartTime  = ktime_get();

	char usb_path[32];
	strcpy(card->driver, "micro lite");
	strcpy(card->shortname, "micro lite");
	usb_make_path(microlite->Device, usb_path, sizeof(usb_path));
	snprintf(microlite->Card->longname, sizeof(microlite->Card->longname),
	         "MOTU micro lite (serial %s), at %s, %s speed", 
	         microlite->Device->serial ? microlite->Device->serial : "?",
			 usb_path,
	         microlite->Device->speed == USB_SPEED_HIGH ? "high" : "full");

	//printk(KERN_INFO "Sound Card device created: %s\n", microlite->Card->longname);

	microlite->Interface  = usb_ifnum_to_if(microlite->Device, 1);
	if (!microlite->Interface)
	{
		dev_err(&microlite->Device->dev, "interface 1 not found\n");
		err = -ENXIO;
		goto probe_error;
	}
	err = usb_driver_claim_interface(&motu_driver, microlite->Interface, microlite);
	if (err != 0)
	{
		dev_err(&microlite->Device->dev, "Failed to claim interface 1\n");
		goto probe_error;
	}

	//INIT_LIST_HEAD(&microlite->DeviceList);
	spin_lock_init(&microlite->Lock);
	mutex_init(&microlite->Mutex);
	spin_lock_init(&microlite->DisconnectLock);
	init_rwsem(&microlite->DisconnectSemaphore);
	mutex_init(&microlite->SubstreamStateMutex);
	timer_setup(&microlite->ErrorTimer, MicroliteErrorTimer, 0);
	//INIT_WORK(&microlite->MidiOutWork, MicroliteMidiOutWork);

	memset(&microlite->Endpoints, 0, sizeof(microlite->Endpoints));
	// 1: INT IN, ISOCH OUT
	// 2: INT IN, INT OUT
	// 3: BULK IN, BULK OUT
#if PIPE_TYPE == PIPE_BULK
	err = usb_set_interface(microlite->Device, 1, 3);
#elif PIPE_TYPE == PIPE_INTERRUPT
	err = usb_set_interface(microlite->Device, 1, 2);
#elif PIPE_TYPE == PIPE_ISOCHRONOUS
	err = usb_set_interface(microlite->Device, 1, 1);
#else
#  error Invalid pipe type.
#endif
	//if (microlite->Interface->num_altsetting <= 3)
	if (err != 0)
	{
		dev_err(&microlite->Device->dev, "Couldn't set alt setting for interface 1\n");
		//err = -ENOENT;
		goto probe_error;
	}

	//printk(KERN_INFO "Creating RawMidi devices for sound card\n");
	err = snd_rawmidi_new(card, "USB MIDI", 0, 5, 5, &microlite->RawMidi);
	if (err < 0)
		goto probe_error;
	//printk(KERN_INFO "RawMidi devices created OK\n");
	

	strcpy(microlite->RawMidi->name, card->shortname);
	microlite->RawMidi->info_flags = SNDRV_RAWMIDI_INFO_OUTPUT | SNDRV_RAWMIDI_INFO_INPUT | SNDRV_RAWMIDI_INFO_DUPLEX;
	microlite->RawMidi->ops = &s_MotuMidiGlobalOps;
	microlite->RawMidi->private_data = microlite;
	microlite->RawMidi->private_free = MicroliteRawMidiFree;
	snd_rawmidi_set_ops(microlite->RawMidi, SNDRV_RAWMIDI_STREAM_OUTPUT, &s_MotuMidiOutOps);
	snd_rawmidi_set_ops(microlite->RawMidi, SNDRV_RAWMIDI_STREAM_INPUT, &s_MotuMidiInOps);

	if (err < 0)
		goto probe_error;

	err = CreateOutEndpoint(microlite, 2);
	if (err < 0)
		goto probe_error;

	err = CreateInEndpoint(microlite, 0x81);
	if (err < 0)
		goto probe_error;


	for (j = 0; j < 5; ++j)
	{
		CreateSubstream(microlite,
					    SNDRV_RAWMIDI_STREAM_OUTPUT,
					    j,
					    &microlite->Endpoints.Out->Ports[j].Substream);
		CreateSubstream(microlite,
					    SNDRV_RAWMIDI_STREAM_INPUT,
					    j,
					    &microlite->Endpoints.In->Ports[j].Substream);
	}

	//printk(KERN_INFO "created 5 output and 5 input ports\n");

	if (err < 0)
		goto probe_error;
	usb_autopm_get_interface_no_resume(microlite->Interface);

	// If we're smarter, `Microlite_t` will be global to the driver, and
	// we'll allocate data to each individual Microlite (I only have one, so
	// this is a non-issue, but bad practice to keep it all global), whereupon
	// we'll use `list_add_tail` to add the device structures to the list,
	// to be iterated when the driver is stopped, etc.
	//list_add_tail(&microlite->MidiListLinkage, midi_list);
	//return 0;

	//err = usb_driver_claim_interface(&motu_driver, microlite->Interface, microlite);
	//if (err < 0)
	//{
	//	microlite->intf = NULL;
	//	err = -EBUSY;
	//	goto probe_error;
	//}

	err = snd_card_register(card);
	if (err < 0)
		goto probe_error;

	usb_set_intfdata(interface, microlite);
	s_devicesUsed |= 1 << cardIndex;

	//mutex_unlock(&s_devicesMutex);
	return 0;

probe_error:
	FreeUsbResources(microlite, interface);
	snd_card_free(card);
	//mutex_unlock(&s_devicesMutex);
	return err;
};

static int CreateInEndpoint(struct Microlite_t *microlite, int endpointId /* , int endpointInterval */ )
{
	struct MicroliteEndpointIn_t *endpoint;
	void *buffer;
	unsigned int pipe;
	int length;
	unsigned int i;
	int err;

	//printk(KERN_INFO "Creating IN endpoint\n");
	microlite->Endpoints.In = NULL;

	endpoint = kzalloc(sizeof(struct MicroliteEndpointIn_t), GFP_KERNEL);
	if (!endpoint)
		return -ENOMEM;
	endpoint->Microlite = microlite;

	for (i = 0; i < INPUT_URBS; ++i)
	{
		endpoint->Urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!endpoint->Urbs[i])
		{
			err = -ENOMEM;
			goto error;
		}
	}
#if PIPE_TYPE == PIPE_INTERRUPT
	pipe = usb_rcvintpipe(microlite->Device, endpointId); // INT endpoint
#elif PIPE_TYPE == PIPE_BULK
	pipe = usb_rcvbulkpipe(microlite->Device, endpointId); // BULK endpoint
#else
#  error Invalid pipe type.
#endif
	length = usb_maxpacket(microlite->Device, pipe);
	for (i = 0; i < INPUT_URBS; ++i)
	{
		buffer = usb_alloc_coherent(microlite->Device, length, GFP_KERNEL,
					    &endpoint->Urbs[i]->transfer_dma);
		if (!buffer)
		{
			err = -ENOMEM;
			goto error;
		}
#if PIPE_TYPE == PIPE_INTERRUPT
		usb_fill_int_urb(endpoint->Urbs[i], microlite->Device, // INT endpoint
				 pipe, buffer, length,
				 OnInUrbCompleted,
				 endpoint, 1);
#elif PIPE_TYPE == PIPE_BULK
		usb_fill_bulk_urb(endpoint->Urbs[i], microlite->Device,
					  pipe, buffer, length,
					  OnInUrbCompleted, endpoint);
#else
#  error Invalid pipe type.
#endif
		endpoint->Urbs[i]->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
		err = usb_urb_ep_type_check(endpoint->Urbs[i]);
		if (err < 0)
		{
			dev_err(&microlite->Device->dev, "invalid MIDI in EP %x\n", endpointId);
			goto error;
		}
	}

	microlite->Endpoints.In = endpoint;
	return 0;

 error:
	DeleteInEndpoint(endpoint);
	return err;
};

/*
 * Frees an input endpoint.
 * May be called when ep hasn't been initialized completely.
 */
static void DeleteInEndpoint(struct MicroliteEndpointIn_t *endpoint)
{
	unsigned int i;

	for (i = 0; i < INPUT_URBS; ++i)
		if (endpoint->Urbs[i])
			FreeUrb(endpoint->Microlite, endpoint->Urbs[i],
					    endpoint->Urbs[i]->transfer_buffer_length);
	kfree(endpoint);
};

static void DeleteOutEndpoint(struct MicroliteEndpointOut_t *endpoint)
{
	ClearOutEndpoint(endpoint);
	kfree(endpoint);
};

/*
 * Frees an output endpoint.
 * May be called when ep hasn't been initialized completely.
 */
static void ClearOutEndpoint(struct MicroliteEndpointOut_t *endpoint)
{
	unsigned int i;

	for (i = 0; i < OUTPUT_URBS; ++i)
		if (endpoint->Urbs[i].Urb)
		{
			FreeUrb(endpoint->Microlite, endpoint->Urbs[i].Urb,
					    endpoint->MaxTransfer);
			endpoint->Urbs[i].Urb = NULL;
		}
}


static void FreeUrb(struct Microlite_t *microlite, struct urb *urb, unsigned int buffer_length)
{
	usb_free_coherent(microlite->Device, buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
	usb_free_urb(urb);
};

static void FreeUsbResources(struct Microlite_t* microlite, struct usb_interface* interface)
{
	struct usb_interface* intf = microlite->Interface;
	microlite->Interface = NULL;
	if (intf)
	{
		usb_set_intfdata(intf, NULL);
		//if (intf != interface)
		//	usb_driver_release_interface(&motu_driver, intf);
	}
};


static void motu_disconnect_handler(struct usb_interface *interface)
{
	struct Microlite_t* microlite = usb_get_intfdata(interface);
	struct list_head* midi;
	if (!microlite)
		return;

	guard(mutex)(&s_devicesMutex);
	//mutex_lock(&s_devicesMutex);
	//set_bit(DISCONNECTED, &microlite->State);
	snd_card_disconnect(microlite->Card);
	//list_for_each(midi, &microlite->MidiListLinkage)
	//	MicroliteDisconnect(midi);
	MicroliteDisconnect(microlite);
	FreeUsbResources(microlite, interface);
	s_devicesUsed &= ~(1 << microlite->CardIndex);
	snd_card_free_when_closed(microlite->Card);
	//mutex_unlock(&s_devicesMutex);
};

/*
 * Unlinks all URBs (must be done before the usb_device is deleted).
 */
static void MicroliteDisconnect(/*struct list_head *p*/ struct Microlite_t* microlite)
{
	//struct Microlite_t *microlite;
	unsigned int i, j;

	//microlite = list_entry(p, struct Microlite_t, MidiListLinkage);
	/*
	 * an URB's completion handler may start the timer and
	 * a timer may submit an URB. To reliably break the cycle
	 * a flag under lock must be used
	 */
	down_write(&microlite->DisconnectSemaphore);
	spin_lock_irq(&microlite->DisconnectLock);
	microlite->Disconnected = 1;
	spin_unlock_irq(&microlite->DisconnectLock);
	up_write(&microlite->DisconnectSemaphore);

	del_timer_sync(&microlite->ErrorTimer);

	if (microlite->Endpoints.Out)
		cancel_work_sync(&microlite->Endpoints.Out->Work);

	if (microlite->Endpoints.Out)
	{
		for (j = 0; j < 5; ++j)
			del_timer_sync(&microlite->Endpoints.Out->Ports[j].Timer);
		for (j = 0; j < OUTPUT_URBS; ++j)
			usb_kill_urb(microlite->Endpoints.Out->Urbs[j].Urb);
		// finish_out_endpoint(microlite->Endpoints.Out);
		microlite->Endpoints.Out->ActiveUrbs = 0;
		if (microlite->Endpoints.Out->DrainUrbs)
		{
			microlite->Endpoints.Out->DrainUrbs = 0;
			wake_up(&microlite->Endpoints.Out->DrainWait);
		}
	}
	if (microlite->Endpoints.In)
		for (j = 0; j < INPUT_URBS; ++j)
			usb_kill_urb(microlite->Endpoints.In->Urbs[j]);

	/* free endpoints here; later call can result in Oops */
	if (microlite->Endpoints.Out)
		ClearOutEndpoint(microlite->Endpoints.Out);
	if (microlite->Endpoints.In)
	{
		DeleteInEndpoint(microlite->Endpoints.In);
		microlite->Endpoints.In = NULL;
	}
}


/*
 * Processes the data read from the device.
 */
static void OnInUrbCompleted(struct urb *urb)
{
	struct MicroliteEndpointIn_t *endpoint = urb->context;
	struct MicroliteInPort_t *port;
	struct Microlite_t* microlite;
	int i, p;
	uint8_t deviceMask;
	const uint8_t *buffer, *bufferEnd;
	uint8_t value;


	if (urb->status != 0)
	{
		int err = UrbError(urb);
		if (err < 0)
		{
			if (err != -ENODEV)
			{
				endpoint->ErrorResubmit = 1;
				mod_timer(&endpoint->Microlite->ErrorTimer, jiffies + ERROR_DELAY_JIFFIES);
			}
			return;
		}
		goto Resubmit;
	}


	if (urb->actual_length < 2)
		goto Resubmit; // do nothing; no data

	//printk(KERN_INFO "OnInUrbCompleted: Status %d, %d bytes\n", urb->status, urb->actual_length);
	//DumpUrb("received", urb->transfer_buffer, urb->actual_length);

	microlite = endpoint->Microlite;
	bufferEnd = &((const uint8_t*)urb->transfer_buffer)[urb->actual_length];
	for (buffer = &((const uint8_t*)urb->transfer_buffer)[2]; buffer < bufferEnd; )
	{
		deviceMask = *buffer++;
		if (!deviceMask)
			continue;
		for (p = 0; p < 5 && buffer < bufferEnd; ++p, deviceMask >>= 1)
		{
			if (!(deviceMask & 1))
				continue;

			port = &endpoint->Ports[p];
			value = *buffer++;
			if (!port->Substream)
			{
				printk(KERN_INFO "unexpected port %d!\n", p);
				continue;
			}
			if (!test_bit(port->Substream->number, &microlite->InputTriggered))
				continue;
			snd_rawmidi_receive(port->Substream, &value, 1);
		}
	}

Resubmit:
	urb->dev = endpoint->Microlite->Device;
	SubmitUrb(urb, GFP_ATOMIC);
};


/* called after transfers had been interrupted due to some USB error */
static void MicroliteErrorTimer(struct timer_list *t)
{
	struct Microlite_t *microlite = from_timer(microlite, t, ErrorTimer);
	unsigned int i, j;

	spin_lock(&microlite->DisconnectLock);
	if (microlite->Disconnected)
	{
		spin_unlock(&microlite->DisconnectLock);
		return;
	}

	struct MicroliteEndpointIn_t* in = microlite->Endpoints.In;
	if (in && in->ErrorResubmit)
	{
		in->ErrorResubmit = 0;
		for (j = 0; j < INPUT_URBS; ++j)
		{
			if (atomic_read(&in->Urbs[j]->use_count))
				continue;
			in->Urbs[j]->dev = microlite->Device;
			SubmitUrb(in->Urbs[j], GFP_ATOMIC);
		}
	}
	if (microlite->Endpoints.Out)
		MidiDoOutput(microlite->Endpoints.Out);

	spin_unlock(&microlite->DisconnectLock);
}


static int  MidiOutput_Open(struct snd_rawmidi_substream *substream)
{
	struct Microlite_t *microlite = substream->rmidi->private_data;
	struct MicroliteOutPort_t *port = NULL;
	int i, j;

	//printk(KERN_INFO "MidiOutput_Open\n");

	if (microlite->Endpoints.Out)
		for (j = 0; j < 0x10; ++j)
			if (microlite->Endpoints.Out->Ports[j].Substream == substream)
			{
				port = &microlite->Endpoints.Out->Ports[j];
				break;
			}
	if (!port)
		return -ENXIO;

	substream->runtime->private_data = port;

	down_read(&microlite->DisconnectSemaphore);
	if (microlite->Disconnected)
	{
		up_read(&microlite->DisconnectSemaphore);
		return -ENODEV;
	}

	mutex_lock(&microlite->SubstreamStateMutex);
	microlite->SubstreamOpenCount[0]++;
	mutex_unlock(&microlite->SubstreamStateMutex);

	up_read(&microlite->DisconnectSemaphore);
	return 0;

};

static int  MidiOutput_Close(struct snd_rawmidi_substream *substream)
{
	struct MicroliteOutPort_t *port = substream->runtime->private_data;
	struct Microlite_t *microlite = substream->rmidi->private_data;
	int i, j;

	//printk(KERN_INFO "MidiOutput_Close\n");

	cancel_work_sync(&port->Endpoint->Work);

	down_read(&microlite->DisconnectSemaphore);
	if (microlite->Disconnected)
		goto Return_Semaphore;

	mutex_lock(&microlite->SubstreamStateMutex);
	microlite->SubstreamOpenCount[0]--;
	mutex_unlock(&microlite->SubstreamStateMutex);

Return_Semaphore:
	up_read(&microlite->DisconnectSemaphore);
	return 0;
};

static void MidiOutput_Trigger(struct snd_rawmidi_substream *substream, int up)
{
	struct MicroliteOutPort_t *port = (struct MicroliteOutPort_t*)substream->runtime->private_data;

	//printk(KERN_INFO "MidiOutput_Trigger(substream #%d, up=%d)\n", substream->number, up);
	port->Active = up;
	if (up)
	{
		if (port->Endpoint->Microlite->Disconnected)
		{
			/* gobble up remaining bytes to prevent wait in snd_rawmidi_drain_output */
			snd_rawmidi_proceed(substream);
			return;
		}
		queue_work(system_highpri_wq, &port->Endpoint->Work);
	}
};

static void MidiOutput_Drain(struct snd_rawmidi_substream *substream)
{
	struct MicroliteOutPort_t *port = substream->runtime->private_data;
	struct MicroliteEndpointOut_t *endpoint = port->Endpoint;
	unsigned int drainUrbs;
	DEFINE_WAIT(wait);
	long timeout;
	ktime_t timeNow;

	//printk(KERN_INFO "MidiOutput_Drain(substream #%d)\n", substream->number);

	if (endpoint->Microlite->Disconnected)
		return;

	timeout = msecs_to_jiffies(5);

	//if (port->Active && port->TimeWhenTransferWillFinish >
	/*
	 * The substream buffer is empty, but some data might still be in the
	 * currently active URBs, so we have to wait for those to complete.
	 */
	spin_lock_irq(&endpoint->BufferLock);
	timeNow = ktime_get_ns();
	if (port->TimeWhenTransferWillFinish > timeNow)
		timeout = usecs_to_jiffies(ktime_sub_ns(port->TimeWhenTransferWillFinish, timeNow) / 1000);
	drainUrbs = endpoint->ActiveUrbs;
	//printk("Draining: %ld jiffies, ActiveUrbs=%02Xh\n", timeout, drainUrbs);
	if (drainUrbs)
	{
		endpoint->DrainUrbs |= drainUrbs;
		do
		{
			prepare_to_wait(&endpoint->DrainWait, &wait, TASK_UNINTERRUPTIBLE);
			spin_unlock_irq(&endpoint->BufferLock);
			timeout = schedule_timeout(timeout);
			spin_lock_irq(&endpoint->BufferLock);
			drainUrbs &= endpoint->DrainUrbs;
		} while (drainUrbs && timeout);
		finish_wait(&endpoint->DrainWait, &wait);
		endpoint->DrainUrbs = 0;
	}
	port->Active = 0;
	//printk("Draining complete\n");
	spin_unlock_irq(&endpoint->BufferLock);
};

static int  MidiInput_Open(struct snd_rawmidi_substream *substream)
{
	struct Microlite_t *microlite = substream->rmidi->private_data;
	unsigned int i;
	unsigned long flags;

	//printk(KERN_INFO "MidiInput_Open\n");

	down_read(&microlite->DisconnectSemaphore);
	if (microlite->Disconnected)
	{
		up_read(&microlite->DisconnectSemaphore);
		return -ENODEV;
	}

	mutex_lock(&microlite->SubstreamStateMutex);
	microlite->SubstreamOpenCount[1]++;
	if (microlite->SubstreamOpenCount[1] == 0 || microlite->InputRunning)
		goto Return;

	if (microlite->Endpoints.In)
	{
		for (i = 0; i < INPUT_URBS; ++i)
		{
			struct urb *urb = microlite->Endpoints.In->Urbs[i];
			spin_lock_irqsave(&microlite->DisconnectLock, flags);
			if (!atomic_read(&urb->use_count))
			{
				urb->dev = microlite->Device;
				SubmitUrb(urb, GFP_ATOMIC);
			}
			spin_unlock_irqrestore(&microlite->DisconnectLock, flags);
		}
	}

	microlite->InputRunning = 1;


Return:
	mutex_unlock(&microlite->SubstreamStateMutex);
	up_read(&microlite->DisconnectSemaphore);
	return 0;
};

static int  MidiInput_Close(struct snd_rawmidi_substream *substream)
{
	unsigned int j;
	struct Microlite_t *microlite = substream->rmidi->private_data;

	//printk(KERN_INFO "MidiInput_Close\n");

	down_read(&microlite->DisconnectSemaphore);
	if (microlite->Disconnected)
		goto Return_Semaphore;

	mutex_lock(&microlite->SubstreamStateMutex);

	microlite->SubstreamOpenCount[1]--;
	if (microlite->SubstreamOpenCount[1] == 0 && microlite->InputRunning)
	{
		if (microlite->Endpoints.In)
			for (j = 0; j < INPUT_URBS; ++j)
				usb_kill_urb(microlite->Endpoints.In->Urbs[j]);
	
		microlite->InputRunning = 0;
	}

	mutex_unlock(&microlite->SubstreamStateMutex);
Return_Semaphore:
	up_read(&microlite->DisconnectSemaphore);
	return 0;

};

static void MidiInput_Trigger(struct snd_rawmidi_substream *substream, int up)
{
	struct Microlite_t *microlite = substream->rmidi->private_data;

	//printk(KERN_INFO "MidiInput_Trigger\n");

	if (up)
		set_bit(substream->number, &microlite->InputTriggered);
	else
		clear_bit(substream->number, &microlite->InputTriggered);
};

static void Microlite_GetPortInfo(struct snd_rawmidi *rmidi, int number, struct snd_seq_port_info *seq_port_info)
{
	//printk(KERN_INFO "Microlite_GetPortInfo\n");
	seq_port_info->type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC
	                    | SNDRV_SEQ_PORT_TYPE_HARDWARE
	                    | SNDRV_SEQ_PORT_TYPE_PORT;
};

static void MicroliteRawMidiFree(struct snd_rawmidi* rawmidi)
{
	struct Microlite_t *microlite = rawmidi->private_data;
	struct MicroliteEndpoint_t *endpoint = &microlite->Endpoints;
	int i;

	if (endpoint->Out)
	{
		for (i = 0; i < OUTPUT_URBS; ++i)
			if (endpoint->Out->Urbs[i].Urb)
			{
				usb_free_coherent(microlite->Device,
				                  endpoint->Out->MaxTransfer,
				                  endpoint->Out->Urbs[i].Urb->transfer_buffer,
				                  endpoint->Out->Urbs[i].Urb->transfer_dma);
				usb_free_urb(endpoint->Out->Urbs[i].Urb);
				endpoint->Out->Urbs[i].Urb = NULL;
			}
		kfree(endpoint->Out);
		endpoint->Out = NULL;
	}

	if (endpoint->In)
	{
		for (i = 0; i < INPUT_URBS; ++i)
			if (endpoint->In->Urbs[i])
			{
				usb_free_coherent(microlite->Device,
				                  endpoint->In->Urbs[i]->transfer_buffer_length,
				                  endpoint->In->Urbs[i]->transfer_buffer,
				                  endpoint->In->Urbs[i]->transfer_dma);
				usb_free_urb(endpoint->In->Urbs[i]);
				endpoint->In->Urbs[i] = NULL;
			}
		kfree(endpoint->In);
		endpoint->In = NULL;
	}

	mutex_destroy(&microlite->SubstreamStateMutex);
	//kfree(microlite);
};

/*
 * Creates an output endpoint, and initializes output ports.
 */
static int CreateOutEndpoint(struct Microlite_t *microlite, int endpointNum)
{
	struct MicroliteEndpointOut_t *endpoint;
	unsigned int i;
	unsigned int pipe;
	uint8_t *buffer;
	int err;

	//printk(KERN_INFO "Creating OUT endpoint\n");

	microlite->Endpoints.Out = NULL;
	endpoint = kzalloc(sizeof(*endpoint), GFP_KERNEL);
	if (!endpoint)
		return -ENOMEM;
	endpoint->Microlite = microlite;

	//printk(KERN_INFO "Allocating OUT URBs\n");
	for (i = 0; i < OUTPUT_URBS; ++i)
	{
		endpoint->Urbs[i].Urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!endpoint->Urbs[i].Urb)
		{
			err = -ENOMEM;
			goto error;
		}
		endpoint->Urbs[i].Endpoint = endpoint;
		endpoint->Urbs[i].Urb->context = &endpoint->Urbs[i];
	}

	//printk(KERN_INFO "Initialising transfer buffers for OUT URBs\n");
#if PIPE_TYPE == PIPE_INTERRUPT
	pipe = usb_sndintpipe(microlite->Device, endpointNum); // INT endpoint
#elif PIPE_TYPE == PIPE_BULK
	pipe = usb_sndbulkpipe(microlite->Device, endpointNum);   // BULK endpoint
#else
#  error Invalid pipe type.
#endif
	endpoint->MaxTransfer = usb_maxpacket(microlite->Device, pipe);
	for (i = 0; i < OUTPUT_URBS; ++i)
	{
		buffer = usb_alloc_coherent(microlite->Device,
		                            endpoint->MaxTransfer,
		                            GFP_KERNEL,
		                            &endpoint->Urbs[i].Urb->transfer_dma);
		if (!buffer)
		{
			err = -ENOMEM;
			dev_err(&microlite->Device->dev, "Failed to create buffer for OUT URB #%d\n", i);
			goto error;
		}
#if PIPE_TYPE == PIPE_INTERRUPT
		usb_fill_int_urb(endpoint->Urbs[i].Urb,            // INT endpoint
		                 microlite->Device,
		                 pipe,
		                 buffer,
		                 endpoint->MaxTransfer,
		                 OnOutUrbCompleted,
		                 &endpoint->Urbs[i],
		                 1);
#elif PIPE_TYPE == PIPE_BULK
		usb_fill_bulk_urb(endpoint->Urbs[i].Urb,
		                  microlite->Device,
		                  pipe,
		                  buffer,
		                  endpoint->MaxTransfer,
		                  OnOutUrbCompleted,
		                  &endpoint->Urbs[i]);
#else
#  error Invalid pipe type.
#endif
		err = usb_urb_ep_type_check(endpoint->Urbs[i].Urb);
		if (err < 0)
		{
			dev_err(&microlite->Device->dev, "invalid MIDI out EP %x\n", endpointNum);
			goto error;
		}
		buffer[0] = 0xA0;
		buffer[1] = 0;
		endpoint->Urbs[i].Urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
	}

	spin_lock_init(&endpoint->BufferLock);
	INIT_WORK(&endpoint->Work, MicroliteMidiOutWork);
	init_waitqueue_head(&endpoint->DrainWait);
	init_waitqueue_head(&endpoint->OverflowWait);

	for (i = 0; i < 5; ++i)
	{
		endpoint->Ports[i].Endpoint = endpoint;
		endpoint->Ports[i].Cable = i;
		timer_setup(&endpoint->Ports[i].Timer, OnPortTransmissionFinished, 0);
	}

	//if (microlite->usb_protocol_ops->init_out_endpoint)
	//	microlite->usb_protocol_ops->init_out_endpoint(endpoint);

	microlite->Endpoints.Out = endpoint;
	return 0;

 error:
	DeleteOutEndpoint(endpoint);
	return err;
};

static void OnPortTransmissionFinished(struct timer_list * timer)
{
	struct MicroliteOutPort_t *port = from_timer(port, timer, Timer);
	struct MicroliteEndpointOut_t *endpoint = port->Endpoint;
	u64 timeNow = ktime_get_ns();

	guard(spinlock_irqsave)(&endpoint->BufferLock);

	if (port->TimeWhenTransferWillFinish > timeNow)
		return;

	endpoint->PortsWaiting &= ~(1<<port->Cable);
};


static void OnOutUrbCompleted(struct urb *urb)
{
	struct OutUrbContext_t *context = urb->context;
	struct MicroliteEndpointOut_t *endpoint = context->Endpoint;
	unsigned int urbIndex;
	unsigned long flags;

	//printk(KERN_INFO "OnOutUrbCompleted: Status=%d, %d bytes\n", urb->status, urb->actual_length);

	spin_lock_irqsave(&endpoint->BufferLock, flags);
	urbIndex = context - endpoint->Urbs;
	endpoint->ActiveUrbs &= ~(1 << urbIndex);
	//if (unlikely(endpoint->DrainUrbs))
	//{
	//	endpoint->DrainUrbs &= ~(1 << urbIndex);
	//	wake_up(&endpoint->DrainWait);
	//}
	spin_unlock_irqrestore(&endpoint->BufferLock, flags);

	if (urb->status < 0)
	{
		int err = UrbError(urb);
		if (err < 0)
		{
			if (err != -ENODEV)
				mod_timer(&endpoint->Microlite->ErrorTimer, jiffies + ERROR_DELAY_JIFFIES);
			return;
		}
	}

	MidiDoOutput(endpoint);
};

static void CreateSubstream(struct Microlite_t *microlite,
				       int stream, int number, 
				       struct snd_rawmidi_substream **rsubstream)
{
	struct port_info *port_info;
	const char *name_format;
	struct usb_interface *intf;
	struct usb_host_interface *hostif;
	uint8_t jack_name_buf[32];
	uint8_t *default_jack_name = "MIDI";
	uint8_t *jack_name = default_jack_name;
	uint8_t iJack;
	int res;
	struct snd_rawmidi_substream *substream;

	list_for_each_entry(substream, &microlite->RawMidi->streams[stream].substreams, list)
	{
		if (substream->number == number)
			goto GotSubstream;
	}
	dev_err(&microlite->Device->dev, "substream %d:%d not found\n", stream, number);
	return;

GotSubstream:
	snprintf(substream->name, sizeof(substream->name),
		 "%s %s %d", microlite->Card->shortname, jack_name, number + 1);

	*rsubstream = substream;
}

/*
 * Submits the URB, with error handling.
 */
static int SubmitUrb(struct urb *urb, gfp_t flags)
{
	int err = usb_submit_urb(urb, flags);
	if (err < 0 && err != -ENODEV)
		dev_err(&urb->dev->dev, "usb_submit_urb: %d\n", err);
	return err;
};


static int UrbError(const struct urb *urb)
{
	switch (urb->status) {
	/* manually unlinked, or device gone */
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
	case -ENODEV:
		return -ENODEV;
	/* errors that might occur during unplugging */
	case -EPROTO:
	case -ETIME:
	case -EILSEQ:
		return -EIO;
	default:
		dev_err(&urb->dev->dev, "urb status %d\n", urb->status);
		return 0; /* continue */
	}
};

static void DumpUrb(const char *type, const u8 *data, int length)
{
	pr_debug("%s packet: [", type);
	for (; length > 0; ++data, --length)
		pr_cont(" %02x", *data);
	pr_cont(" ]\n");
};

/* table of devices that work with this driver */
static const struct usb_device_id motu_ids[] = {
	{ USB_DEVICE(0x07fd, 0x0001) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, motu_ids);

static struct usb_driver motu_driver = {
	.name = "snd-microlite",
	.id_table = motu_ids,
	.probe = motu_probe_handler,
	.disconnect = motu_disconnect_handler,
#if 0
	.suspend = motu_suspend,
	.resume = motu_resume,
#endif
};
module_usb_driver(motu_driver);



/* 
 * snd_usbmidi_create: Creates a structure for holding the USB MIDI abstraction (snd_usb_midi).
 * This structure is a list element by way of its umidi->list field.
 * This structure is added to a given list (list_add_tail(&umidi->list, [[arg]] midi_list))
 * In the case of the UA-101, the UA-101 has its own UA-specific structure (ua101) which can also
 *   be a list element because of its ua->midi_list field.
 * The UA-101 initialises the midi_list header, and passes this to snd_usbmidi_create, linking the
 *   UA-101 UA-specific structure with its USB MIDI abstract structure.
 * Later, this midi_list is iterated when a device is disconnected. ua101_disconnect is called, which
 *   iterates ua->midi_list and calls snd_usbmidi_disconnect on each linked item, of type snd_usb_midi.
 * Hence, ua->midi_list is a driver-global list that holds snd_usb_midi devices.
 * */
