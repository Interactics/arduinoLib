/************************************
 * Designed by Interactics
 * Functionality : RS485 Packet Read
 * 
 * Modifying Dynamixel SDK
 * DATE : 2021.01.05
 ************************************/

#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

#define MAX_TX_SIZE          14

/***** ReadPacket*******/
#define HEAD1     0
#define HEAD2     1
#define HEAD3     2
#define RVRS      3
#define DXID      4
#define LENGHTH_1 5
#define LENGHTH_2 6
#define INST      7
#define PARAM_1   8
#define PARAM_2   9
#define PARAM_3   10
#define PARAM_4   11
#define CRC_1     12
#define CRC_2     13

/******* Packet Result ********/
#define COMM_FAIL -99
#define COMM_SUCCESS      1

/******* Ram Address *********/
#define PRESENTVELOCITY  128

/******* Dynamixle SETUP ********/
#define DyID1 1
#define DyBAUDRATE     1000000 // [bps]
#define DyDELAYTIME    500     // [us]
#define DyPACKETWAIT  ((DyBAUDRATE/1,000,000)/8) // [us]

#define Serial485   Serial3
#define SerialDebug Serial


unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  unsigned short crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

int readPacket(const uint8_t aID, uint8_t aInst, uint8_t aSize, uint32_t* data) {
  // 667 micros
  
  uint8_t txPacket[MAX_TX_SIZE] = {0};
  unsigned short crc;

  txPacket[HEAD1]     = 0xFF;
  txPacket[HEAD2]     = 0xFF;
  txPacket[HEAD3]     = 0xFD;
  txPacket[RVRS]      = 0x00;
  txPacket[DXID]      = aID;
  txPacket[LENGHTH_1] = 0x07; // Read Packet size == 7byte
  txPacket[LENGHTH_2] = 0x00;
  txPacket[INST]      = 0x02; // Read
  txPacket[PARAM_1]   = DXL_LOBYTE(aInst);    // INST. What you want to read.
  txPacket[PARAM_2]   = DXL_HIBYTE(aInst);    // INST. What you want to read.
  txPacket[PARAM_3]   = DXL_LOBYTE(aSize);    // SIZE LO,
  txPacket[PARAM_4]   = DXL_HIBYTE(aSize);    // SIZE HI,

  crc = update_crc(0, txPacket, MAX_TX_SIZE - 2);

  txPacket[CRC_1]   = DXL_LOBYTE(crc);
  txPacket[CRC_2]   = DXL_HIBYTE(crc);

  ///// Write Packet
  Serial485.write(txPacket, MAX_TX_SIZE);
  Serial485.flush();

  /**** Read txPacket ****/
  //  for (int idx = 0; idx < MAX_TX_SIZE ; idx++) SerialDebug.print(txPacket[idx], HEX);
  //  SerialDebug.println();
  /***********************/
  uint8_t rxPacket_size = 9 + aSize + 2;
  uint8_t* rxPacket     = (uint8_t*)malloc(rxPacket_size);

  int rx_length;

  // Waiting time. while the time that packet is responed.
  unsigned long wait_start = micros();
  while (micros() - wait_start  <  DyPACKETWAIT * rxPacket_size + DyDELAYTIME) { } 
  // Baudrate [bps]/ 8 [bit/Byte] /1,000,000 [us/sec] * PacketSize [byte] + DynamixelDelayTime  == [us]

  rx_length = Serial485.available();
  for (int i = 0; i < rxPacket_size; i++) {
    rxPacket[i] = Serial485.read();
  }

  // Cleaning Buffer
  while (Serial485.available()) {
    Serial485.read();
  }

  if (rxPacket[HEAD1] == 0xFF && rxPacket[HEAD2] == 0xFF && rxPacket[HEAD3] == 0xFD) {
    // pass
  }
  else {
    // Communication Fail
    free(rxPacket);
    return COMM_FAIL;
  }

  /**** Read rxPacket ****/
  //  for (int idx = 0; idx < rxPacket_size ; idx++) SerialDebug.print(rxPacket[idx], HEX);
  //  SerialDebug.println();
  /***********************/

  if (aSize == 4)
    *data = DXL_MAKEDWORD(DXL_MAKEWORD(rxPacket[rxPacket_size - 6], rxPacket[rxPacket_size - 5]), DXL_MAKEWORD(rxPacket[rxPacket_size - 4], rxPacket[rxPacket_size - 3]));
  else if (aSize == 2)
    *data = DXL_MAKEWORD(rxPacket[rxPacket_size - 4], rxPacket[rxPacket_size - 3]);
  else if (aSize == 1)
    *data = rxPacket[rxPacket_size - 3];

  free(rxPacket);
  return COMM_SUCCESS;
}

int readVelocity(uint8_t DxID, uint32_t* pData) {
  return readPacket(DxID, PRESENTVELOCITY, 4, pData);
}

void setup() {
  Serial485.begin(DyBAUDRATE);
  SerialDebug.begin(9600);
}

void loop() {
  uint32_t data = 0;
  int result;
  result = readVelocity(DyID1, &data);
  if (result != COMM_FAIL) {
    SerialDebug.print("value : ");
    SerialDebug.println(int(data));
  }
  else if (result == COMM_FAIL) {
    SerialDebug.println("Comm Fail!");
  }
}
