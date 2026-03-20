#include "stdlib.h"
#include <stdint.h>
#include <string.h>
#include "vofa_Debug.h"

#define VOFA_DATA_SIZE 10
float vofa_data[VOFA_DATA_SIZE];
float last_vofa_data[VOFA_DATA_SIZE];

float vofa_GetData(uint8_t *data)
{
  static float num = 0.0f;

  if (data == NULL)
    {
      return num;
    }
  // Frame format: 0x5A + 4-byte payload + '\n' (0x0A).
  if ((data[0] == 0x5A||data[0] == 0x5B||data[0] == 0x5C) && (data[5] == 0x0A))
    {
      uint8_t is_ascii_payload =
        ((data[1] >= 0x20) && (data[1] <= 0x7E)) &&
        ((data[2] >= 0x20) && (data[2] <= 0x7E)) &&
        ((data[3] >= 0x20) && (data[3] <= 0x7E)) &&
        ((data[4] >= 0x20) && (data[4] <= 0x7E));

      if (is_ascii_payload)
        {
          char buf[5];
          char *endptr;
          float parsed;
          memcpy(buf, &data[1], 4);
          buf[4] = '\0';
          parsed = strtof(buf, &endptr);
          if (endptr == buf)
            {
              return num;
            }
          num = parsed;
        }
      else
        {
          uint32_t raw = ((uint32_t)data[1]) |
                         ((uint32_t)data[2] << 8) |
                         ((uint32_t)data[3] << 16) |
                         ((uint32_t)data[4] << 24);
          memcpy(&num, &raw, sizeof(num));
        }
      switch(data[0])
        {
        case 0x5A:
          vofa_data[0] = num;
          break;
        case 0x5B:
          vofa_data[1] = num;
          break;
          case 0x5C:
          vofa_data[2] = num;
          break;
        }

    }
  return num;
}

void vofa_data_write(uint8_t index, float *receiver)
{
  if (receiver == NULL)
    {
      return;
    }

  if (index >= VOFA_DATA_SIZE)
    {
      *receiver = 0.0f;
      return;
    }

  if (last_vofa_data[index] != vofa_data[index])
    {
      last_vofa_data[index] = vofa_data[index];
      *receiver = last_vofa_data[index];
    }

  return;
}















