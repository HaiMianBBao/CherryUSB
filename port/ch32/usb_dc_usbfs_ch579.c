#include "usb_dc.h"
#include "usbd_core.h"
#include "usb_ch579_reg.h"

/*!< ep dir in */
#define EP_DIR_IN 1

/*!< ep dir out */
#define EP_DIR_OUT 0

/*!< 8-bit value of endpoint control register */
#define EPn_CTRL(epid) \
  *(volatile uint8_t *)(0x40008022 + epid * 4)

/*!< The length register value of the endpoint send buffer */
#define EPn_TX_LEN(epid) \
  *(volatile uint8_t *)(0x40008020 + epid * 4)

/*!< get ep dir by epadd */
#define GET_EP_DIR(ep_add) (uint8_t)(ep_add & 0x80)

/*!< read setup packet to use in ep0 in */
#define GET_SETUP_PACKET(data_add) \
  *(struct usb_setup_packet *)data_add

/*!< set epid ep tx valid */
#define EPn_SET_TX_VALID(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_T_RES | UEP_T_RES_ACK;

/*!< set epid ep rx valid */
#define EPn_SET_RX_VALID(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_R_RES | UEP_R_RES_ACK;

/*!< set epid ep tx nak */
#define EPn_SET_TX_NAK(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_T_RES | UEP_T_RES_NAK;

/*!< set epid ep rx nak */
#define EPn_SET_RX_NAK(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_R_RES | UEP_R_RES_NAK;

/*!< set epid ep tx stall */
#define EPn_SET_TX_STALL(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_T_RES | UEP_T_RES_STALL

/*!< set epid ep rx stall */
#define EPn_SET_RX_STALL(epid) \
  EPn_CTRL(epid) = EPn_CTRL(epid) & ~MASK_UEP_R_RES | UEP_R_RES_STALL

/*!< set epid ep tx len */
#define EPn_SET_TX_LEN(epid, len) \
  EPn_TX_LEN(epid) = len

/*!< get epid ep rx len */
#define EPn_GET_RX_LEN(epid) \
  R8_USB_RX_LEN

/*!< ep nums */
#define EP_NUMS 5
/*!< ep mps */
#define EP_MPS 64
/*!< set ep4 in mps 64 */
#define EP4_IN_MPS EP_MPS
/*!< set ep4 out mps 64 */
#define EP4_OUT_MPS EP_MPS

/*!< User defined assignment endpoint RAM */
__attribute__((aligned(4))) uint8_t ep0_data_buff[64 + 64 + 64]; /*!< ep0(64)+ep4_out(64)+ep4_in(64) */
__attribute__((aligned(4))) uint8_t ep1_data_buff[64 + 64];      /*!< ep1_out(64)+ep1_in(64) */
__attribute__((aligned(4))) uint8_t ep2_data_buff[64 + 64];      /*!< ep2_out(64)+ep2_in(64) */
__attribute__((aligned(4))) uint8_t ep3_data_buff[64 + 64];      /*!< ep3_out(64)+ep3_in(64) */

/**
 * @brief   Endpoint information structure
 */
typedef struct _usbd_ep_info
{
  uint8_t mps;          /*!< Maximum packet length of endpoint */
  uint8_t eptype;       /*!< Endpoint Type */
  uint8_t *ep_ram_addr; /*!< Endpoint buffer address */

  uint8_t ep_enable; /* Endpoint enable */
  uint8_t *xfer_buf;
  uint32_t xfer_len;
  uint32_t actual_xfer_len;
} usbd_ep_info;

static struct _ch579_core_prvi
{
  uint8_t address; /*!< Address */
  usbd_ep_info ep_in[EP_NUMS];
  usbd_ep_info ep_out[EP_NUMS];
  struct usb_setup_packet setup;
} usb_dc_cfg;

__WEAK void usb_dc_low_level_init(void)
{
}

__WEAK void usb_dc_low_level_deinit(void)
{
}

/**
 * @brief            Set address
 * @pre              None
 * @param[in]        address ：8-bit valid address
 * @retval           >=0 success otherwise failure
 */
int usbd_set_address(const uint8_t address)
{
  if (address == 0)
  {
    R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | address;
  }
  usb_dc_cfg.address = address;
  return 0;
}

/**
 * @brief            Open endpoint
 * @pre              None
 * @param[in]        ep_cfg : Endpoint configuration structure pointer
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_open(const struct usbd_endpoint_cfg *ep_cfg)
{
  uint8_t epid = USB_EP_GET_IDX(ep_cfg->ep_addr);
  uint8_t mps = ep_cfg->ep_mps;
  if (epid > (EP_NUMS - 1))
  {
    /*!< Endpoint number overflow */
    return -1;
  }

  if (USB_EP_DIR_IS_IN(ep_cfg->ep_addr))
  {
    /*!< in */
    usb_dc_cfg.ep_in[epid].mps = mps;
    usb_dc_cfg.ep_in[epid].ep_enable = true;
  }
  else if (USB_EP_DIR_IS_OUT(ep_cfg->ep_addr))
  {
    /*!< out */
    usb_dc_cfg.ep_out[epid].mps = mps;
    usb_dc_cfg.ep_out[epid].ep_enable = true;
  }
  return 0;
}

/**
 * @brief            Close endpoint
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_close(const uint8_t ep)
{
  /*!< ep id */
  uint8_t epid = USB_EP_GET_IDX(ep);
  /*!< update ep max packet length */
  if (USB_EP_DIR_IS_IN(ep))
  {
    /*!< in */
    usb_dc_cfg.ep_in[epid].ep_enable = false;
  }
  else if (USB_EP_DIR_IS_OUT(ep))
  {
    /*!< out */
    usb_dc_cfg.ep_out[epid].ep_enable = false;
  }
  return 0;
}

/**
 * @brief Setup in ep transfer setting and start transfer.
 *
 * This function is asynchronous.
 * This function is similar to uart with tx dma.
 *
 * This function is called to write data to the specified endpoint. The
 * supplied usbd_endpoint_callback function will be called when data is transmitted
 * out.
 *
 * @param[in]  ep        Endpoint address corresponding to the one
 *                       listed in the device configuration table
 * @param[in]  data      Pointer to data to write
 * @param[in]  data_len  Length of the data requested to write. This may
 *                       be zero for a zero length status packet.
 * @return 0 on success, negative errno code on fail.
 */
int usbd_ep_start_write(const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
  uint8_t ep_idx = USB_EP_GET_IDX(ep);

  if (!data && data_len)
  {
    return -1;
  }
  if (!usb_dc_cfg.ep_in[ep_idx].ep_enable)
  {
    return -2;
  }
  if ((uint32_t)data & 0x03)
  {
    return -3;
  }

  usb_dc_cfg.ep_in[ep_idx].xfer_buf = (uint8_t *)data;
  usb_dc_cfg.ep_in[ep_idx].xfer_len = data_len;
  usb_dc_cfg.ep_in[ep_idx].actual_xfer_len = 0;

  if (data_len == 0)
  {
    /*!< write 0 len data */
    EPn_SET_TX_LEN(ep_idx, 0);
    /*!< enable tx */
    EPn_SET_TX_VALID(ep_idx);
    /*!< return */
    return 0;
  }
  else
  {
    /*!< Not zlp */
    data_len = MIN(data_len, usb_dc_cfg.ep_in[ep_idx].mps);
    /*!< write buff */
    memcpy(usb_dc_cfg.ep_in[ep_idx].ep_ram_addr, data, data_len);
    /*!< write real_wt_nums len data */
    EPn_SET_TX_LEN(ep_idx, data_len);
    /*!< enable tx */
    EPn_SET_TX_VALID(ep_idx);
  }
  return 0;
}

/**
 * @brief Setup out ep transfer setting and start transfer.
 *
 * This function is asynchronous.
 * This function is similar to uart with rx dma.
 *
 * This function is called to read data to the specified endpoint. The
 * supplied usbd_endpoint_callback function will be called when data is received
 * in.
 *
 * @param[in]  ep        Endpoint address corresponding to the one
 *                       listed in the device configuration table
 * @param[in]  data      Pointer to data to read
 * @param[in]  data_len  Max length of the data requested to read.
 *
 * @return 0 on success, negative errno code on fail.
 */
int usbd_ep_start_read(const uint8_t ep, uint8_t *data, uint32_t data_len)
{
  uint8_t ep_idx = USB_EP_GET_IDX(ep);
  if (!data && data_len)
  {
    return -1;
  }
  if (!usb_dc_cfg.ep_out[ep_idx].ep_enable)
  {
    return -2;
  }
  if ((uint32_t)data & 0x03)
  {
    return -3;
  }

  usb_dc_cfg.ep_out[ep_idx].xfer_buf = (uint8_t *)data;
  usb_dc_cfg.ep_out[ep_idx].xfer_len = data_len;
  usb_dc_cfg.ep_out[ep_idx].actual_xfer_len = 0;

  if (data_len == 0)
  {
    return 0;
  }
  else
  {
    data_len = MIN(data_len, usb_dc_cfg.ep_out[ep_idx].mps);
  }
  return 0;
}

/**
 * @brief            Endpoint setting stall
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_set_stall(const uint8_t ep)
{
  /*!< ep id */
  uint8_t epid = USB_EP_GET_IDX(ep);
  EPn_SET_RX_STALL(epid);
  EPn_SET_TX_STALL(epid);
  return 0;
}

/**
 * @brief            Endpoint clear stall
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_clear_stall(const uint8_t ep)
{
  int ret;
  switch (ep)
  {
  case 0x82:
    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
    ret = 0;
    break;
  case 0x02:
    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
    ret = 0;
    break;
  case 0x81:
    R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
    ret = 0;
    break;
  case 0x01:
    R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
    ret = 0;
    break;
  default:
    /*!< Unsupported endpoint */
    ret = -1;
    break;
  }
  return ret;
}

/**
 * @brief            Check endpoint status
 * @pre              None
 * @param[in]        ep ： Endpoint address
 * @param[out]       stalled ： Outgoing endpoint status
 * @retval           >=0 success otherwise failure
 */
int usbd_ep_get_stall(const uint8_t ep, uint8_t *stalled)
{
  return 0;
}

int usb_dc_deinit(void)
{
  //TODO: 
  return 0;
}

/**
 * @brief            USB initialization
 * @pre              None
 * @param[in]        None
 * @retval           >=0 success otherwise failure
 */
int usb_dc_init(void)
{
  usb_dc_cfg.ep_in[0].ep_ram_addr = ep0_data_buff;
  usb_dc_cfg.ep_out[0].ep_ram_addr = ep0_data_buff;

  usb_dc_cfg.ep_in[1].ep_ram_addr = ep1_data_buff + 64;
  usb_dc_cfg.ep_out[1].ep_ram_addr = ep1_data_buff;

  usb_dc_cfg.ep_in[2].ep_ram_addr = ep2_data_buff + 64;
  usb_dc_cfg.ep_out[2].ep_ram_addr = ep2_data_buff;

  usb_dc_cfg.ep_in[3].ep_ram_addr = ep3_data_buff + 64;
  usb_dc_cfg.ep_out[3].ep_ram_addr = ep3_data_buff;

  usb_dc_cfg.ep_in[4].ep_ram_addr = ep0_data_buff + 128;
  usb_dc_cfg.ep_out[4].ep_ram_addr = ep0_data_buff + 64;

  /*!< Set the mode first and cancel RB_UC_CLR_ALL */
  R8_USB_CTRL = 0x00;

  R8_UEP4_1_MOD = RB_UEP4_RX_EN | RB_UEP4_TX_EN | RB_UEP1_RX_EN | RB_UEP1_TX_EN;
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN;

  R16_UEP0_DMA = (uint16_t)(uint32_t)ep0_data_buff;
  R16_UEP1_DMA = (uint16_t)(uint32_t)ep1_data_buff;
  R16_UEP2_DMA = (uint16_t)(uint32_t)ep2_data_buff;
  R16_UEP3_DMA = (uint16_t)(uint32_t)ep3_data_buff;

  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  R8_USB_DEV_AD = 0x00;
  /*!< Start the USB device and DMA, and automatically return to NAK before the interrupt flag is cleared during the interrupt */
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
  /*!< Clear interrupt flag */
  R8_USB_INT_FG = 0xFF;

  R16_PIN_ANALOG_IE |= RB_PIN_USB_IE;
  R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;

  usb_dc_low_level_init();
  return 0;
}

/**
 * @brief            USB interrupt processing function
 * @pre              None
 * @param[in]        None
 * @retval           None
 */
void USB_IRQHandler(void)
{
  uint8_t intflag;
  intflag = R8_USB_INT_FG;
  if (intflag & RB_UIF_TRANSFER)
  {
    uint32_t len;
    uint8_t reg = R8_USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP);
    switch (reg & MASK_UIS_TOKEN) /*!< Analyze operation token and endpoint number */
    {
    case UIS_TOKEN_SETUP:
      R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
      /*!< Get setup packet */
      usb_dc_cfg.setup = GET_SETUP_PACKET(usb_dc_cfg.ep_out[0].ep_ram_addr);
      if (usb_dc_cfg.setup.bmRequestType >> USB_REQUEST_DIR_SHIFT == 0)
      {
        /**
         * Ep0 The next in must be the status stage.
         * The device must reply to the host data 0 length packet.
         * Here, set the transmission length to 0 and the transmission status to ACK,
         * and wait for the host to send the in token to retrieve
         */
        R8_UEP0_T_LEN = 0;
        EPn_SET_TX_VALID(0);
      }
      usbd_event_ep0_setup_complete_handler((uint8_t *)&(usb_dc_cfg.setup));
      /*!< enable ep0 rx */
      EPn_SET_RX_VALID(0);
      break;
    case UIS_TOKEN_IN:
      if (!(reg & 0x0F))
      {
        /*!< EP0 */
        switch (usb_dc_cfg.setup.bmRequestType >> USB_REQUEST_DIR_SHIFT)
        {
        case 1:
          /*!< Get */
          R8_UEP0_CTRL ^= RB_UEP_T_TOG;
          /**
           * Here is to take away the last data, and the IN interrupt will be triggered only after it is successfully taken away.
           * Therefore, the status of the in endpoint is set to NAK here. If there is data transmission,
           * the endpoint status will be set to ack again in the in handler of EP0.
           */
          /*!< Set ep0 tx nak */
          EPn_SET_TX_NAK(0);
          /*!< IN */
          if (usb_dc_cfg.ep_in[0].xfer_len > usb_dc_cfg.ep_in[0].mps)
          {
            usb_dc_cfg.ep_in[0].xfer_len -= usb_dc_cfg.ep_in[0].mps;
            usb_dc_cfg.ep_in[0].actual_xfer_len += usb_dc_cfg.ep_in[0].mps;
            usbd_event_ep_in_complete_handler(0 | 0x80, usb_dc_cfg.ep_in[0].actual_xfer_len);
          }
          else
          {
            usb_dc_cfg.ep_in[0].actual_xfer_len += usb_dc_cfg.ep_in[0].xfer_len;
            usb_dc_cfg.ep_in[0].xfer_len = 0;
            usbd_event_ep_in_complete_handler(0 | 0x80, usb_dc_cfg.ep_in[0].actual_xfer_len);
          }
          break;
        case 0:
          /*!< set */
          switch (usb_dc_cfg.setup.bRequest)
          {
          case USB_SET_ADDRESS:
            /*!< Fill in the equipment address */
            R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | usb_dc_cfg.address;
            /*!< No data returned T-NACK */
            /**
             * When the host sends the in token,
             * the data in FIFO has been taken away.
             * Ch58x USB IP needs to manually set the status of the in endpoint to NAK
             */
            R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
            break;
          default:
            /*!< PRINT("state over \n"); */
            /**
             * The host has taken away 0-length packets.
             * Here, you only need to set the status of the endpoint to NAK
             */
            R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
            break;
          }
          break;
        }
      }
      else
      {
        /*!< Others EP */
        if ((reg & 0x07) == 0x04)
        {
          R8_UEP4_CTRL ^= RB_UEP_T_TOG;
        }
        uint8_t epid = (reg & 0x07);
        EPn_SET_TX_NAK(epid);
        if (usb_dc_cfg.ep_in[epid].xfer_len > usb_dc_cfg.ep_in[epid].mps)
        {
          /*!< Need start in again */
          usb_dc_cfg.ep_in[epid].xfer_buf += usb_dc_cfg.ep_in[epid].mps;
          usb_dc_cfg.ep_in[epid].xfer_len -= usb_dc_cfg.ep_in[epid].mps;
          usb_dc_cfg.ep_in[epid].actual_xfer_len += usb_dc_cfg.ep_in[epid].mps;
          if (usb_dc_cfg.ep_in[epid].xfer_len > usb_dc_cfg.ep_in[epid].mps)
          {
            memcpy(usb_dc_cfg.ep_in[epid].ep_ram_addr, usb_dc_cfg.ep_in[epid].xfer_buf, usb_dc_cfg.ep_in[epid].mps);
          }
          else
          {
            memcpy(usb_dc_cfg.ep_in[epid].ep_ram_addr, usb_dc_cfg.ep_in[epid].xfer_buf, usb_dc_cfg.ep_in[epid].xfer_len);
          }
          EPn_SET_TX_VALID(epid);
        }
        else
        {
          usb_dc_cfg.ep_in[epid].actual_xfer_len += usb_dc_cfg.ep_in[epid].xfer_len;
          usb_dc_cfg.ep_in[epid].xfer_len = 0;
          usbd_event_ep_in_complete_handler(epid | 0x80, usb_dc_cfg.ep_in[epid].actual_xfer_len);
        }
      }
      break;
    case UIS_TOKEN_OUT:
      if (!(reg & 0x0F))
      {
        /*!< ep0 out */
        R8_UEP0_CTRL ^= RB_UEP_R_TOG;
        uint32_t read_count = EPn_GET_RX_LEN(0);
        memcpy(usb_dc_cfg.ep_out[0].xfer_buf, usb_dc_cfg.ep_out[0].ep_ram_addr, read_count);

        usb_dc_cfg.ep_out[0].actual_xfer_len += read_count;
        usb_dc_cfg.ep_out[0].xfer_len -= read_count;
        usbd_event_ep_out_complete_handler(0x00, usb_dc_cfg.ep_out[0].actual_xfer_len);
        EPn_SET_RX_VALID(0);
      }
      else
      {
        /*!< Others EP */
        if ((reg & 0x0F) == 0x04)
        {
          R8_UEP4_CTRL ^= RB_UEP_R_TOG;
        }

        uint8_t epid = (reg & 0x0F);
        uint32_t read_count = EPn_GET_RX_LEN(epid);
        memcpy(usb_dc_cfg.ep_out[epid].xfer_buf, usb_dc_cfg.ep_out[epid].ep_ram_addr, read_count);
        usb_dc_cfg.ep_out[epid].xfer_buf += read_count;
        usb_dc_cfg.ep_out[epid].actual_xfer_len += read_count;
        usb_dc_cfg.ep_out[epid].xfer_len -= read_count;

        if ((read_count < usb_dc_cfg.ep_out[epid].mps) || (usb_dc_cfg.ep_out[epid].xfer_len == 0))
        {
          usbd_event_ep_out_complete_handler(((epid)&0x7f), usb_dc_cfg.ep_out[epid].actual_xfer_len);
        }
        else
        {
          EPn_SET_RX_VALID(epid);
        }
      }
      break;
    default:
      break;
    }
    R8_USB_INT_FG = RB_UIF_TRANSFER;
  }
  else if (intflag & RB_UIF_BUS_RST)
  {
    R8_USB_DEV_AD = 0;
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

    R8_USB_INT_FG = RB_UIF_BUS_RST;

    /*!< Call the reset callback in the protocol stack to register the endpoint callback function */
    usbd_event_reset_handler();
  }
  else if (intflag & RB_UIF_SUSPEND)
  {
    if (R8_USB_MIS_ST & RB_UMS_SUSPEND)
    {
      /*!< Suspend */
    }
    else
    {
      /*!< Wake up */
    }
    R8_USB_INT_FG = RB_UIF_SUSPEND;
  }
  else
  {
    R8_USB_INT_FG = intflag;
  }
}
