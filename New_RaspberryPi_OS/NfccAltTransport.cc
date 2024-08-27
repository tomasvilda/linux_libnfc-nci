/******************************************************************************
 *  Copyright 2021 NXP
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include <errno.h>
#include <fcntl.h>
#ifdef ANDROID
#include <hardware/nfc.h>
#endif
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <NfccI2cTransport.h>
#include <NfccAltTransport.h>
#include <phNfcStatus.h>
#include <phNxpLog.h>
#include <string.h>
#include "phNxpNciHal_utils.h"

//NXP Test new Raspberry pi os
#include <gpiod.h>
//NXP End Test new Raspberry pi os

#define CRC_LEN 2
#define NORMAL_MODE_HEADER_LEN 3
#define FW_DNLD_HEADER_LEN 2
#define FW_DNLD_LEN_OFFSET 1
#define NORMAL_MODE_LEN_OFFSET 2
#define FRAGMENTSIZE_MAX PHNFC_I2C_FRAGMENT_SIZE
extern phTmlNfc_i2cfragmentation_t fragmentation_enabled;
extern phTmlNfc_Context_t* gpphTmlNfc_Context;

//NXP Test new Raspberry pi os
const char *chipname = "gpiochip0";
struct gpiod_chip *chip;
struct gpiod_line *VEN_line;
struct gpiod_line *IRQ_line;
struct gpiod_line *FWDNLD_line;
//NXP End Test new Raspberry pi os

NfccAltTransport::NfccAltTransport() {
  iEnableFd = 0;
  iInterruptFd = 0;
}

/*******************************************************************************
**
** Function         Flushdata
**
** Description      Reads payload of FW rsp from NFCC device into given buffer
**
** Parameters       pDevHandle - valid device handle
**                  pBuffer    - buffer for read data
**                  numRead    - number of bytes read by calling function
**
** Returns          always returns -1
**
*******************************************************************************/
int NfccAltTransport::Flushdata(void* pDevHandle, uint8_t* pBuffer,
                                int numRead) {
  int retRead = 0;
  uint16_t totalBtyesToRead =
      pBuffer[FW_DNLD_LEN_OFFSET] + FW_DNLD_HEADER_LEN + CRC_LEN;
  /* we shall read totalBtyesToRead-1 as one byte is already read by calling
   * function*/
  retRead = read((intptr_t)pDevHandle, pBuffer + numRead, totalBtyesToRead - 1);
  if (retRead > 0) {
    numRead += retRead;
    phNxpNciHal_print_packet("RECV", pBuffer, numRead);
  } else if (retRead == 0) {
    NXPLOG_TML_E("%s _i2c_read() [pyld] EOF", __func__);
  } else {
    if (bFwDnldFlag == false) {
      NXPLOG_TML_D("%s _i2c_read() [hdr] received", __func__);
      phNxpNciHal_print_packet("RECV", pBuffer - numRead,
                               NORMAL_MODE_HEADER_LEN);
    }
    NXPLOG_TML_E("%s _i2c_read() [pyld] errno : %x", __func__, errno);
  }
  SemPost();
  return -1;
}

/*******************************************************************************
**
** Function         Reset
**
** Description      Reset NFCC device, using VEN pin
**
** Parameters       pDevHandle     - valid device handle
**                  eType          - reset level
**
** Returns           0   - reset operation success
**                  -1   - reset operation failure
**
*******************************************************************************/
int NfccAltTransport::NfccReset(void* pDevHandle, NfccResetType eType) {
  int ret = -1;
  NXPLOG_TML_D("%s, VEN eType %ld", __func__, eType);

  if (NULL == pDevHandle) {
    return -1;
  }
  switch (eType) {
    case MODE_POWER_OFF:
      gpio_set_fwdl(0);
      gpio_set_ven(0);
      break;
    case MODE_POWER_ON:
      gpio_set_fwdl(0);
      gpio_set_ven(1);
      break;
    case MODE_FW_DWNLD_WITH_VEN:
      gpio_set_fwdl(1);
      gpio_set_ven(0);
      gpio_set_ven(1);
      break;
    case MODE_FW_DWND_HIGH:
      gpio_set_fwdl(1);
      break;
    case MODE_POWER_RESET:
      gpio_set_ven(0);
      gpio_set_ven(1);
      break;
    case MODE_FW_GPIO_LOW:
      gpio_set_fwdl(0);
      break;
    default:
      NXPLOG_TML_E("%s, VEN eType %ld", __func__, eType);
      return -1;
  }
  if ((eType != MODE_FW_DWNLD_WITH_VEN) && (eType != MODE_FW_DWND_HIGH)) {
    EnableFwDnldMode(false);
  }
  if ((eType == MODE_FW_DWNLD_WITH_VEN) || (eType == MODE_FW_DWND_HIGH)) {
    EnableFwDnldMode(true);
  }

  return ret;
}

/*******************************************************************************
**
** Function         GetNfcState
**
** Description      Get NFC state
**
** Parameters       pDevHandle     - valid device handle
** Returns           0   - unknown
**                   1   - FW DWL
**                   2 	 - NCI
**
*******************************************************************************/
int NfccAltTransport::GetNfcState(void* pDevHandle) {
  int ret = NFC_STATE_UNKNOWN;
  NXPLOG_TML_D("%s ", __func__);
  if (NULL == pDevHandle) {
    return ret;
  }
  ret = ioctl((intptr_t)pDevHandle, NFC_GET_NFC_STATE);
  NXPLOG_TML_D("%s :nfc state = %d", __func__, ret);
  return ret;
}
/*******************************************************************************
**
** Function         EnableFwDnldMode
**
** Description      updates the state to Download mode
**
** Parameters       True/False
**
** Returns          None
*******************************************************************************/
void NfccAltTransport::EnableFwDnldMode(bool mode) { bFwDnldFlag = mode; }

/*******************************************************************************
**
** Function         IsFwDnldModeEnabled
**
** Description      Returns the current mode
**
** Parameters       none
**
** Returns           Current mode download/NCI
*******************************************************************************/
bool_t NfccAltTransport::IsFwDnldModeEnabled(void) { return bFwDnldFlag; }

/*******************************************************************************
**
** Function         SemPost
**
** Description      sem_post 2c_read / write
**
** Parameters       none
**
** Returns          none
*******************************************************************************/
void NfccAltTransport::SemPost() {
  int sem_val = 0;
  sem_getvalue(&mTxRxSemaphore, &sem_val);
  if (sem_val == 0) {
    sem_post(&mTxRxSemaphore);
  }
}

/*******************************************************************************
**
** Function         SemTimedWait
**
** Description      Timed sem_wait for avoiding i2c_read & write overlap
**
** Parameters       none
**
** Returns          Sem_wait return status
*******************************************************************************/
int NfccAltTransport::SemTimedWait() {
  NFCSTATUS status = NFCSTATUS_FAILED;
  long sem_timedout = 500 * 1000 * 1000;
  int s = 0;
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 0;
  ts.tv_nsec += sem_timedout;
  while ((s = sem_timedwait(&mTxRxSemaphore, &ts)) == -1 && errno == EINTR) {
    continue; /* Restart if interrupted by handler */
  }
  if (s != -1) {
    status = NFCSTATUS_SUCCESS;
  } else if (errno == ETIMEDOUT && s == -1) {
    NXPLOG_TML_E("%s :timed out errno = 0x%x", __func__, errno);
  }
  return status;
}

void NfccAltTransport::gpio_set_ven(int value) {
  //NXP Test new Raspberry pi os
  if (value == 0) {
    gpiod_line_set_value(VEN_line, 0);
  } else {
    gpiod_line_set_value(VEN_line, 1);
  }
  usleep(10 * 1000);
  //NXP End Test new Raspberry pi os
}

void NfccAltTransport::gpio_set_fwdl(int value) {
  //NXP Test new Raspberry pi os
  if (value == 0) {
     gpiod_line_set_value(FWDNLD_line, 0);
  } else {
    gpiod_line_set_value(FWDNLD_line, 1);
  }
  usleep(10 * 1000);
  //NXP End Test new Raspberry pi os
}

void NfccAltTransport::wait4interrupt(void) {
  //NXP Test new Raspberry pi os
  while(gpiod_line_get_value(IRQ_line) != 1){};
  //NXP End Test new Raspberry pi os
}

/*****************************************************************************
   **
   ** Function         ConfigurePin
   **
   ** Description      Configure Pins such as IRQ, VEN, Firmware Download
   **
   ** Parameters       none
   **
   ** Returns           NFCSTATUS_SUCCESS - on Success/ -1 on Failure
   ****************************************************************************/
int NfccAltTransport::ConfigurePin()
{
  //NXP Test new Raspberry pi os
  chip = gpiod_chip_open_by_name(chipname);
  VEN_line = gpiod_chip_get_line(chip, PIN_ENABLE);
  IRQ_line = gpiod_chip_get_line(chip, PIN_INT);
  FWDNLD_line = gpiod_chip_get_line(chip, PIN_FWDNLD);

  gpiod_line_request_output(VEN_line, "VEN pin", 1);
  gpiod_line_request_output(FWDNLD_line, "FWDNLD pin", 1);
  gpiod_line_request_input(IRQ_line, "IRQ pin");
  //NXP End Test new Raspberry pi os
  return NFCSTATUS_SUCCESS;
}
