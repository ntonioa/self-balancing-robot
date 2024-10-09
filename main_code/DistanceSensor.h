unsigned long measEcho() {
  digitalWrite(TRIG_PIN, LOW);
  delay(100);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH);
}