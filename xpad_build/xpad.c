// SPDX-License-Identifier: GPL-2.0
/*
 * Xbox 360 wired controller driver (minimal, for Jetson Orin Nano / 5.15-tegra)
 * Handles: Microsoft Xbox 360 wired (045e:028e)
 * Creates /dev/input/js0 via Linux joystick API
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/input.h>
#include <linux/slab.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("marsdog");
MODULE_DESCRIPTION("Xbox 360 wired controller driver");

#define XPAD_PKT_LEN 32

/* button bitmasks in packet byte 2 */
#define XPAD_BTN_DPAD_UP      BIT(0)
#define XPAD_BTN_DPAD_DOWN    BIT(1)
#define XPAD_BTN_DPAD_LEFT    BIT(2)
#define XPAD_BTN_DPAD_RIGHT   BIT(3)
#define XPAD_BTN_START        BIT(4)
#define XPAD_BTN_BACK         BIT(5)
#define XPAD_BTN_THUMB_L      BIT(6)
#define XPAD_BTN_THUMB_R      BIT(7)

/* button bitmasks in packet byte 3 */
#define XPAD_BTN_A            BIT(0)
#define XPAD_BTN_B            BIT(1)
#define XPAD_BTN_X            BIT(2)
#define XPAD_BTN_Y            BIT(3)
#define XPAD_BTN_LB           BIT(4)
#define XPAD_BTN_RB           BIT(5)
#define XPAD_BTN_GUIDE        BIT(8)   /* bit 0 of byte 4? no — byte3 bit7 on some fw */

struct usb_xpad {
    struct input_dev   *dev;
    struct usb_device  *udev;
    struct urb         *irq_in;
    unsigned char      *idata;
    dma_addr_t          idata_dma;
    char                phys[64];
};

static const struct usb_device_id xpad_table[] = {
    { USB_DEVICE(0x045e, 0x028e) },  /* Xbox 360 wired */
    { USB_DEVICE(0x045e, 0x028f) },  /* Xbox 360 wired (v2) */
    { USB_DEVICE(0x045e, 0x02a1) },  /* Xbox 360 wired (some OEM) */
    {}
};
MODULE_DEVICE_TABLE(usb, xpad_table);

static void xpad_process_packet(struct usb_xpad *xpad, u16 cmd, unsigned char *data)
{
    struct input_dev *dev = xpad->dev;

    if (data[0] != 0x00 || data[1] != 0x14)
        return;   /* not a normal state packet */

    /* D-pad reported as hat axes (axis 6/7), matching standard xpad behavior */
    input_report_abs(dev, ABS_HAT0X,
        (data[2] & XPAD_BTN_DPAD_RIGHT) ?  1 :
        (data[2] & XPAD_BTN_DPAD_LEFT)  ? -1 : 0);
    input_report_abs(dev, ABS_HAT0Y,
        (data[2] & XPAD_BTN_DPAD_DOWN)  ?  1 :
        (data[2] & XPAD_BTN_DPAD_UP)    ? -1 : 0);

    /* buttons byte 2 (non-dpad) */
    input_report_key(dev, BTN_START,      data[2] & XPAD_BTN_START);
    input_report_key(dev, BTN_SELECT,     data[2] & XPAD_BTN_BACK);
    input_report_key(dev, BTN_THUMBL,     data[2] & XPAD_BTN_THUMB_L);
    input_report_key(dev, BTN_THUMBR,     data[2] & XPAD_BTN_THUMB_R);

    /* buttons byte 3 */
    input_report_key(dev, BTN_A,          data[3] & XPAD_BTN_A);
    input_report_key(dev, BTN_B,          data[3] & XPAD_BTN_B);
    input_report_key(dev, BTN_X,          data[3] & XPAD_BTN_X);
    input_report_key(dev, BTN_Y,          data[3] & XPAD_BTN_Y);
    input_report_key(dev, BTN_TL,         data[3] & XPAD_BTN_LB);
    input_report_key(dev, BTN_TR,         data[3] & XPAD_BTN_RB);

    /* triggers: LT/RT as absolute axes */
    input_report_abs(dev, ABS_Z,  data[4]);   /* LT 0-255 */
    input_report_abs(dev, ABS_RZ, data[5]);   /* RT 0-255 */

    /* sticks */
    input_report_abs(dev, ABS_X,  (s16)le16_to_cpup((__le16 *)(data + 6)));
    input_report_abs(dev, ABS_Y, -(s16)le16_to_cpup((__le16 *)(data + 8)));  /* invert Y */
    input_report_abs(dev, ABS_RX, (s16)le16_to_cpup((__le16 *)(data + 10)));
    input_report_abs(dev, ABS_RY,-(s16)le16_to_cpup((__le16 *)(data + 12)));

    input_sync(dev);
}

static void xpad_irq_in(struct urb *urb)
{
    struct usb_xpad *xpad = urb->context;
    int retval;

    switch (urb->status) {
    case 0:
        break;
    case -ECONNRESET:
    case -ENOENT:
    case -ESHUTDOWN:
        return;
    default:
        goto resubmit;
    }

    xpad_process_packet(xpad, 0, xpad->idata);

resubmit:
    retval = usb_submit_urb(urb, GFP_ATOMIC);
    if (retval)
        dev_err(&xpad->udev->dev, "usb_submit_urb failed: %d\n", retval);
}

static int xpad_open(struct input_dev *dev)
{
    struct usb_xpad *xpad = input_get_drvdata(dev);
    return usb_submit_urb(xpad->irq_in, GFP_KERNEL);
}

static void xpad_close(struct input_dev *dev)
{
    struct usb_xpad *xpad = input_get_drvdata(dev);
    usb_kill_urb(xpad->irq_in);
}

static int xpad_probe(struct usb_interface *intf,
                      const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(intf);
    struct usb_xpad *xpad;
    struct input_dev *input_dev;
    struct usb_endpoint_descriptor *ep_irq_in = NULL;
    struct usb_host_interface *iface_desc;
    int i, error;

    iface_desc = intf->cur_altsetting;

    /* find interrupt IN endpoint */
    for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
        struct usb_endpoint_descriptor *ep = &iface_desc->endpoint[i].desc;
        if (usb_endpoint_is_int_in(ep)) {
            ep_irq_in = ep;
            break;
        }
    }
    if (!ep_irq_in) {
        dev_err(&intf->dev, "no interrupt IN endpoint found\n");
        return -ENODEV;
    }

    xpad = kzalloc(sizeof(*xpad), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!xpad || !input_dev) {
        error = -ENOMEM;
        goto fail1;
    }

    xpad->idata = usb_alloc_coherent(udev, XPAD_PKT_LEN,
                                     GFP_KERNEL, &xpad->idata_dma);
    if (!xpad->idata) {
        error = -ENOMEM;
        goto fail1;
    }

    xpad->irq_in = usb_alloc_urb(0, GFP_KERNEL);
    if (!xpad->irq_in) {
        error = -ENOMEM;
        goto fail2;
    }

    xpad->udev = udev;
    xpad->dev  = input_dev;

    usb_make_path(udev, xpad->phys, sizeof(xpad->phys));
    strlcat(xpad->phys, "/input0", sizeof(xpad->phys));

    input_dev->name = "Xbox 360 Wired Controller";
    input_dev->phys = xpad->phys;
    usb_to_input_id(udev, &input_dev->id);
    input_dev->dev.parent = &intf->dev;
    input_dev->open  = xpad_open;
    input_dev->close = xpad_close;
    input_set_drvdata(input_dev, xpad);

    /* buttons: A B X Y LB RB SELECT START LS RS */
    input_set_capability(input_dev, EV_KEY, BTN_A);
    input_set_capability(input_dev, EV_KEY, BTN_B);
    input_set_capability(input_dev, EV_KEY, BTN_X);
    input_set_capability(input_dev, EV_KEY, BTN_Y);
    input_set_capability(input_dev, EV_KEY, BTN_TL);
    input_set_capability(input_dev, EV_KEY, BTN_TR);
    input_set_capability(input_dev, EV_KEY, BTN_SELECT);
    input_set_capability(input_dev, EV_KEY, BTN_START);
    input_set_capability(input_dev, EV_KEY, BTN_THUMBL);
    input_set_capability(input_dev, EV_KEY, BTN_THUMBR);

    /* axes 0-5: sticks + triggers; 6-7: D-pad hat */
    input_set_abs_params(input_dev, ABS_X,     -32768, 32767, 16, 128);  /* LX  axis0 */
    input_set_abs_params(input_dev, ABS_Y,     -32768, 32767, 16, 128);  /* LY  axis1 */
    input_set_abs_params(input_dev, ABS_Z,      0, 255, 0, 0);            /* LT  axis2 */
    input_set_abs_params(input_dev, ABS_RX,    -32768, 32767, 16, 128);  /* RX  axis3 */
    input_set_abs_params(input_dev, ABS_RY,    -32768, 32767, 16, 128);  /* RY  axis4 */
    input_set_abs_params(input_dev, ABS_RZ,     0, 255, 0, 0);            /* RT  axis5 */
    input_set_abs_params(input_dev, ABS_HAT0X, -1, 1, 0, 0);             /* DX  axis6 */
    input_set_abs_params(input_dev, ABS_HAT0Y, -1, 1, 0, 0);             /* DY  axis7 */

    usb_fill_int_urb(xpad->irq_in, udev,
                     usb_rcvintpipe(udev, ep_irq_in->bEndpointAddress),
                     xpad->idata, XPAD_PKT_LEN,
                     xpad_irq_in, xpad,
                     ep_irq_in->bInterval);
    xpad->irq_in->transfer_dma = xpad->idata_dma;
    xpad->irq_in->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    error = input_register_device(input_dev);
    if (error)
        goto fail3;

    usb_set_intfdata(intf, xpad);
    dev_info(&intf->dev, "Xbox 360 controller registered as %s\n",
             input_dev->name);
    return 0;

fail3:
    usb_free_urb(xpad->irq_in);
fail2:
    usb_free_coherent(udev, XPAD_PKT_LEN, xpad->idata, xpad->idata_dma);
fail1:
    input_free_device(input_dev);
    kfree(xpad);
    return error;
}

static void xpad_disconnect(struct usb_interface *intf)
{
    struct usb_xpad *xpad = usb_get_intfdata(intf);
    usb_set_intfdata(intf, NULL);
    if (xpad) {
        usb_kill_urb(xpad->irq_in);
        input_unregister_device(xpad->dev);
        usb_free_urb(xpad->irq_in);
        usb_free_coherent(xpad->udev, XPAD_PKT_LEN,
                          xpad->idata, xpad->idata_dma);
        kfree(xpad);
    }
}

static struct usb_driver xpad_driver = {
    .name       = "xpad",
    .probe      = xpad_probe,
    .disconnect = xpad_disconnect,
    .id_table   = xpad_table,
};

module_usb_driver(xpad_driver);
