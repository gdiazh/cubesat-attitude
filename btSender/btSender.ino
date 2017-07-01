float a;
float b;
uint8_t t;
 
void setup()
{
  Serial3.begin(115200);       // Inicializamos el puerto serie Serial3 que hemos creado
  Serial.begin(115200);   // Inicializamos  el puerto serie
  a = 0;
  b = 0;
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
        test(114.123, -121.056);
        test(102.105, 121.056);
    }
    else
    {
        Serial3.write(cmd);
    }
  }*/
    test(a, b);
    a=100*cos(2*3.1415926*t/100);
    b=100*cos(2*3.1415926*t/100);
    t+=1;
    /*if (a>127) a = 0;
    if (b>127) b = 0;*/
    delay(1);
}

void sendPacket(uint8_t framehead, uint8_t D1, uint8_t D2, uint8_t D3)
{
    uint8_t packet_len = 5;
    uint8_t out_packet[5];
    
     //Initialize
    for (int j=0;j<packet_len;j++) out_packet[j] = 0;
    
    //Build Packet
    out_packet[0] = framehead;   //Frame head
    out_packet[1] = D1;
    out_packet[2] = D2;
    out_packet[3] = D3;
    out_packet[4] = checksum(out_packet, packet_len) & 0x00FF; //Low byte of data checksum;
    
    //Send packet
    for (int j=0;j<packet_len;j++) Serial3.write(out_packet[j]);
}

uint8_t checksum(uint8_t *packet, uint8_t n)
{
    uint32_t sum = 0;
    for (int j=0;j<n-1;j++) sum += packet[j];
    return sum & 0x00ff;
}

void encode(float data[], uint8_t packet[])
{
    uint8_t int1 = (uint8_t) abs(data[0]);
    float decimal1 = (abs(data[0])-int1)*1000;
    if(data[0]>=0)
    {
        packet[0] = abs(int1);
        packet[1] = (uint8_t)abs(decimal1)+1;
    }
    else
    {
        packet[0] = abs(int1)|0b10000000;
        packet[1] = (uint8_t)abs(decimal1);
    }
    uint8_t int2 = (uint8_t) abs(data[1]);
    float decimal2 = (abs(data[1])-int2)*1000;
    if(data[1]>=0)
    {
        packet[2] = abs(int2);
        packet[3] = (uint8_t)abs(decimal2)+1;
    }
    else
    {
        packet[2] = abs(int2)|0b10000000;
        packet[3] = (uint8_t)abs(decimal2);
    }

    packet[4] = checksum(packet, 5);
}

void test(float D1, float D2)
{
    float data[2] = {D1, D2};
    uint8_t packet_test[5] = {0,0,0,0,0};
    encode(data, packet_test);
    sendPacket(packet_test[0], packet_test[1], packet_test[2], packet_test[3]);
    Serial.print("packet_sent = [");
    for (int i = 0; i < 4; i++)
    {
        Serial.print(packet_test[i]);
        Serial.print(",");
    }
    Serial.print(packet_test[4]);
    Serial.println("]");
}