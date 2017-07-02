float a;
float b;
float c;
float d;
uint8_t t;
 
void setup()
{
  Serial3.begin(115200);       // Inicializamos el puerto serie Serial3 que hemos creado
  Serial.begin(115200);   // Inicializamos  el puerto serie
  a = 0;
  b = 0;
  c = 0;
  d = 0;
  t = 0;
}
 
void loop()
{
  /*if(Serial3.available())    // Si llega un dato por el puerto Serial3 se envía al monitor serial
  {
    Serial.write(Serial3.read());
  }
  if(Serial.available())  // Si llega un dato por el monitor serial se envía al puerto Serial3
  {
    char cmd = Serial.read();
    if (cmd == '1')
    {
        // test(114.123, -121.056);
        // test(102.105, 121.056);
        Serial.println("data = [-102.92, -121.14, -51.34, -67.15]");
        test(-102.92, -121.14, -51.34, -67.15);
    }
    else if (cmd == '2')
    {
        Serial.println("data = [102.92, 121.14, 51.34, 67.15]");
        test(102.92, 121.14, 51.34, 67.15);
    }
    else
    {
        Serial3.write(cmd);
    }
  }*/
    test(100*a, 100*b, 100*c, 100*d);
    a=sin(2*3.1415926*t/100);
    b=cos(2*3.1415926*t/100);
    c=a*a;
    d=b*b;
    t+=1;
    delay(1);
}

void sendFrame(uint8_t frame[], uint8_t sz)
{   
    for (int j=0;j<sz;j++) Serial3.write(frame[j]);
}

uint8_t checksum(uint8_t *packet, uint8_t n)
{
    uint32_t sum = 0;
    for (int j=0;j<n-1;j++) sum += packet[j];
    return sum & 0x00FF;
}

void bytesEncode(float number, uint8_t encode_bytes[])
{
    uint16_t int_part = (uint16_t) abs(number);           //[0-65535]
    float decimal_part = (abs(number) - int_part)*100;   //[0-99]

    uint8_t NH = (int_part)>>8;                //Number High Byte
    uint8_t NL = (int_part) & 0x00FF;          //Number Low Byte
    uint8_t D = (int)decimal_part;             //Decimal part (7 bits)
    uint8_t SD = D;                                 //Sign and Decimal Byte
    if (number<=0) SD = D|0b10000000;               //Sign bit

    encode_bytes[0] = NH;
    encode_bytes[1] = NL;
    encode_bytes[2] = SD;
}

void encode(float data[], uint8_t packet[])
{
    uint8_t num1_bytes[3];
    uint8_t num2_bytes[3];
    uint8_t num3_bytes[3];
    uint8_t num4_bytes[3];
    bytesEncode(data[0], num1_bytes);
    bytesEncode(data[1], num2_bytes);
    bytesEncode(data[2], num3_bytes);
    bytesEncode(data[3], num4_bytes);

    packet[0] = num1_bytes[0];
    packet[1] = num1_bytes[1];
    packet[2] = num1_bytes[2];

    packet[3] = num2_bytes[0];
    packet[4] = num2_bytes[1];
    packet[5] = num2_bytes[2];

    packet[6] = num3_bytes[0];
    packet[7] = num3_bytes[1];
    packet[8] = num3_bytes[2];

    packet[9] = num4_bytes[0];
    packet[10] = num4_bytes[1];
    packet[11] = num4_bytes[2];

    packet[12] = checksum(packet, 13);
}

void printFrame(uint8_t frame[], uint8_t sz)
{
    Serial.print("packet_sent = [");
    for (int i = 0; i < sz-1; i++)
    {
        Serial.print(frame[i]);
        Serial.print(",");
    }
    Serial.print(frame[sz-1]);
    Serial.println("]");
}

void test(float D1, float D2, float D3, float D4)
{
    float data[4] = {D1, D2, D3, D4};
    uint8_t frame_test[13];
    encode(data, frame_test);
    sendFrame(frame_test, 13);
    printFrame(frame_test, 13);
}