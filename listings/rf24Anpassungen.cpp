void RF24::SensorknotenIoT_resetRegister(void){
  write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
  flush_rx();
  flush_tx();
  write_register(NRF_CONFIG,(read_register(NRF_CONFIG)) & ~_BV(PRIM_RX));
  write_register(EN_RXADDR, 0x03 );
}