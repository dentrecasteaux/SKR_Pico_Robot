#include <Arduino.h>
#include <unity.h>

#include "TMC2209.h"

void test_drv_status_fields_are_decoded_at_datasheet_positions()
{
  const uint32_t raw = 0x155U | (1UL << 12) | (1UL << 14) | (17UL << 16) |
                       (1UL << 25) | (1UL << 28) | (1UL << 30) |
                       (1UL << 31);
  const TMC2209::Status status = TMC2209::decodeStatus(raw);

  TEST_ASSERT_TRUE(status.connected);
  TEST_ASSERT_EQUAL_UINT16(0x155U, status.stallGuardResult);
  TEST_ASSERT_EQUAL_UINT8(17U, status.currentScale);
  TEST_ASSERT_TRUE(status.shortToSupplyA);
  TEST_ASSERT_FALSE(status.shortToSupplyB);
  TEST_ASSERT_TRUE(status.stealthChop);
  TEST_ASSERT_TRUE(status.overTemperature);
  TEST_ASSERT_FALSE(status.overTemperaturePreWarning);
  TEST_ASSERT_FALSE(status.shortToGroundA);
  TEST_ASSERT_TRUE(status.shortToGroundB);
  TEST_ASSERT_TRUE(status.openLoadB);
  TEST_ASSERT_TRUE(status.standstill);
  TEST_ASSERT_TRUE(status.hasFault());
}

void test_disconnected_status_is_a_fault()
{
  const TMC2209::Status status = TMC2209::decodeStatus(0, false);
  TEST_ASSERT_FALSE(status.connected);
  TEST_ASSERT_TRUE(status.hasFault());
}

void setup()
{
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_drv_status_fields_are_decoded_at_datasheet_positions);
  RUN_TEST(test_disconnected_status_is_a_fault);
  UNITY_END();
}

void loop()
{
}
