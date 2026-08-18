#include <Arduino.h>
#include <unity.h>

#include "TMC2209.h"

void test_drv_status_fields_are_decoded_at_datasheet_positions()
{
  const uint32_t raw = (1UL << 0) | (1UL << 2) | (1UL << 4) |
                       (1UL << 6) | (17UL << 16) | (1UL << 30) |
                       (1UL << 31);
  const TMC2209::Status status = TMC2209::decodeStatus(raw);

  TEST_ASSERT_TRUE(status.connected);
  TEST_ASSERT_EQUAL_UINT16(0U, status.stallGuardResult);
  TEST_ASSERT_EQUAL_UINT8(17U, status.currentScale);
  TEST_ASSERT_TRUE(status.shortToSupplyA);
  TEST_ASSERT_FALSE(status.shortToSupplyB);
  TEST_ASSERT_TRUE(status.stealthChop);
  TEST_ASSERT_FALSE(status.fullStepActive);
  TEST_ASSERT_FALSE(status.stallGuard);
  TEST_ASSERT_FALSE(status.overTemperature);
  TEST_ASSERT_TRUE(status.overTemperaturePreWarning);
  TEST_ASSERT_TRUE(status.shortToGroundA);
  TEST_ASSERT_FALSE(status.shortToGroundB);
  TEST_ASSERT_TRUE(status.openLoadA);
  TEST_ASSERT_FALSE(status.openLoadB);
  TEST_ASSERT_TRUE(status.standstill);
  TEST_ASSERT_TRUE(status.hasFault());
}

void test_clear_stealth_bit_reports_spreadcycle_without_false_faults()
{
  const uint32_t raw = 9UL << 16;
  const TMC2209::Status status = TMC2209::decodeStatus(raw);

  TEST_ASSERT_FALSE(status.stealthChop);
  TEST_ASSERT_EQUAL_UINT8(9U, status.currentScale);
  TEST_ASSERT_FALSE(status.hasFault());
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
  RUN_TEST(test_clear_stealth_bit_reports_spreadcycle_without_false_faults);
  RUN_TEST(test_disconnected_status_is_a_fault);
  UNITY_END();
}

void loop()
{
}
