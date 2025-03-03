#ifndef __FORMAT_CONVERT_H
#define __FORMAT_CONVERT_H

void hsdaoh_unpack_pio_12bit(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info);
void hsdaoh_unpack_pio_12bit_dual(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info);
void hsdaoh_unpack_pio_10bit_iq(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info);
void hsdaoh_unpack_pio_8bit_iq(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info);
void hsdaoh_unpack_pio_pcm1802_audio(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info);
void hsdaoh_unpack_fpga_12bit_dual(hsdaoh_dev_t *dev, hsdaoh_data_info_t *data_info);
#endif
