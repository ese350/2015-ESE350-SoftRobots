#ifndef RFIO_H
#define RFIO_H

void rf_send(char *data, uint8_t len);
int rf_receive(char *data, uint8_t maxLength);

#endif /* RFIO_H */
