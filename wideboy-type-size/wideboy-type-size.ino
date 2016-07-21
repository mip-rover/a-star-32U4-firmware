
void setup() {
  SerialUSB.begin(204300);
}

void loop() {
  // put your main code here, to run repeatedly:
  SerialUSB.print("bool size: ");
  SerialUSB.println(sizeof(bool));
  SerialUSB.print("short size: ");
  SerialUSB.println(sizeof(short));
  SerialUSB.print("int size: ");
  SerialUSB.println(sizeof(int));
  SerialUSB.print("long size: ");
  SerialUSB.println(sizeof(long));
  SerialUSB.print("float size: ");
  SerialUSB.println(sizeof(float));
  SerialUSB.print("double size: ");
  SerialUSB.println(sizeof(double));
}
