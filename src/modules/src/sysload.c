/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * sysload.c - System load monitor
 */

#define DEBUG_MODULE "SYSLOAD"

#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "debug.h"
#include "cfassert.h"
#include "param.h"
#include "log.h"
#include "static_mem.h"

#include "sysload.h"

// Load is measured continuously over this window and published via the "sysload" log group
#define TIMER_PERIOD M2T(500)

static void timerHandler(xTimerHandle timer);

static bool initialized = false;
static uint8_t triggerDump = 0;

typedef struct {
  uint32_t ulRunTimeCounter;
  uint32_t xTaskNumber;
} taskData_t;

#define TASK_MAX_COUNT 32
NO_DMA_CCM_SAFE_ZERO_INIT static taskData_t previousSnapshot[TASK_MAX_COUNT];
NO_DMA_CCM_SAFE_ZERO_INIT static TaskStatus_t taskStats[TASK_MAX_COUNT];
static int taskTopIndex = 0;
static uint32_t previousTotalRunTime = 0;

static StaticTimer_t timerBuffer;

extern uint32_t vl53l8cxRangingUs;

// Tasks with a dedicated log slot. Names are matched on the first
// configMAX_TASK_NAME_LEN - 1 characters since FreeRTOS truncates them
// ("vl53l8cx_get_tof_task" becomes "vl53l8cx_").
enum {
  SLOT_IDLE,
  SLOT_TOF,       // vl53l8cx_get_tof_task (system.c)
  SLOT_DECK,      // vl53l8cx deck task (odometry / wall fit)
  SLOT_SYSLINK,
  SLOT_CRTP_RX,
  SLOT_CRTP_TX,
  SLOT_STAB,
  SLOT_KALMAN,
  SLOT_SENSORS,
  SLOT_APP,
  SLOT_LOG,
  SLOT_TMR,
  SLOT_FLOW,
  SLOT_COUNT
};

static const char* const slotNames[SLOT_COUNT] = {
  "IDLE", "vl53l8cx_", "vl53l8cx", "SYSLINK", "CRTP-RX", "CRTP-TX",
  "STABILIZER", "KALMAN", "SENSORS", "APP", "LOG", "Tmr Svc", "FLOW",
};

// Logged values. Loads are in permille of the window, stacks are free words at peak usage.
static uint16_t slotLoad[SLOT_COUNT];
static uint16_t slotStack[SLOT_COUNT];
static uint16_t cpuLoad;
static uint16_t otherLoad;
static uint16_t stackMin;
static uint8_t taskCountLog;

// Longest time [ms] the idle task did not run. The watchdog is only kicked
// from the idle hook, so a gap of a few hundred ms resets the MCU.
static volatile uint32_t lastIdleTick;
static volatile uint32_t idleGapWindowMax;
static uint16_t idleGapLog;
static uint16_t idleGapPeak;

void sysLoadInit() {
  ASSERT(!initialized);

  xTimerHandle timer = xTimerCreateStatic( "sysLoadMonitorTimer", TIMER_PERIOD, pdTRUE, NULL, timerHandler, &timerBuffer);
  xTimerStart(timer, 100);

  initialized = true;
}

void sysLoadIdleHook(void) {
  uint32_t now = xTaskGetTickCount();
  uint32_t gap = now - lastIdleTick;
  if (gap > idleGapWindowMax) {
    idleGapWindowMax = gap;
  }
  lastIdleTick = now;
}


static taskData_t* getPreviousTaskData(uint32_t xTaskNumber) {
  // Try to find the task in the list of tasks
  for (int i = 0; i < taskTopIndex; i++) {
    if (previousSnapshot[i].xTaskNumber == xTaskNumber) {
      return &previousSnapshot[i];
    }
  }

  // Allocate a new entry
  ASSERT(taskTopIndex < TASK_MAX_COUNT);
  taskData_t* result = &previousSnapshot[taskTopIndex];
  result->xTaskNumber = xTaskNumber;

  taskTopIndex++;

  return result;
}

static int findSlot(const char* taskName) {
  for (int i = 0; i < SLOT_COUNT; i++) {
    if (strncmp(taskName, slotNames[i], configMAX_TASK_NAME_LEN - 1) == 0) {
      return i;
    }
  }
  return -1;
}

static void updateIdleGap(void) {
  // Include a gap that is still in progress
  uint32_t current = xTaskGetTickCount() - lastIdleTick;
  uint32_t gap = idleGapWindowMax;
  if (current > gap) {
    gap = current;
  }
  idleGapWindowMax = 0;

  idleGapLog = gap > UINT16_MAX ? UINT16_MAX : gap;
  if (idleGapLog > idleGapPeak) {
    idleGapPeak = idleGapLog;
  }
}

static void timerHandler(xTimerHandle timer) {
  updateIdleGap();

  uint32_t totalRunTime;
  uint32_t taskCount = uxTaskGetSystemState(taskStats, TASK_MAX_COUNT, &totalRunTime);
  ASSERT(taskCount < TASK_MAX_COUNT);

  // CPU usage is since the previous window, relative to total elapsed time. Time spent in
  // interrupts is included in the task that was interrupted.
  uint32_t totalDelta = totalRunTime - previousTotalRunTime;
  if (totalDelta == 0) {
    totalDelta = 1;
  }

  if (triggerDump != 0) {
    DEBUG_PRINT("Task dump\n");
    DEBUG_PRINT("Load[%%] Stack left[words] Prio Name\n");
  }

  uint32_t otherSum = 0;
  uint16_t minStack = UINT16_MAX;
  for (uint32_t i = 0; i < taskCount; i++) {
    TaskStatus_t* stats = &taskStats[i];
    taskData_t* previousTaskData = getPreviousTaskData(stats->xTaskNumber);

    uint32_t taskRunTime = stats->ulRunTimeCounter;
    uint32_t permille = (uint32_t)(((uint64_t)(taskRunTime - previousTaskData->ulRunTimeCounter) * 1000) / totalDelta);
    if (permille > 1000) {
      permille = 1000;
    }

    int slot = findSlot(stats->pcTaskName);
    if (slot >= 0) {
      slotLoad[slot] = permille;
      slotStack[slot] = stats->usStackHighWaterMark;
    } else {
      otherSum += permille;
    }

    if (stats->usStackHighWaterMark < minStack) {
      minStack = stats->usStackHighWaterMark;
    }

    if (triggerDump != 0) {
      DEBUG_PRINT("%lu.%lu \t%u \t%lu \t%s\n", permille / 10, permille % 10,
                  stats->usStackHighWaterMark, (uint32_t)stats->uxCurrentPriority, stats->pcTaskName);
    }

    previousTaskData->ulRunTimeCounter = taskRunTime;
  }

  otherLoad = otherSum > 1000 ? 1000 : otherSum;
  cpuLoad = 1000 - slotLoad[SLOT_IDLE];
  stackMin = minStack;
  taskCountLog = taskCount;

  previousTotalRunTime = totalRunTime;

  triggerDump = 0;
}


PARAM_GROUP_START(system)

/**
 * @brief Set to nonzero to dump CPU and stack usage to console
 */
PARAM_ADD_CORE(PARAM_UINT8, taskDump, &triggerDump)

PARAM_GROUP_STOP(system)


/**
 * CPU load and stack usage, updated every 500 ms.
 * Loads are in permille (0-1000) of the last window.
 */
LOG_GROUP_START(sysload)
/** @brief Total CPU load (1000 - idle) [permille] */
LOG_ADD(LOG_UINT16, cpu, &cpuLoad)
/** @brief Longest idle starvation during the last window [ms]. Watchdog resets at a few hundred ms */
LOG_ADD(LOG_UINT16, idleGap, &idleGapLog)
/** @brief Longest idle starvation since boot [ms] */
LOG_ADD(LOG_UINT16, idleGapPk, &idleGapPeak)
/** @brief Duration of the latest Gget_Ranging() [us] */
LOG_ADD(LOG_UINT32, rangingUs, &vl53l8cxRangingUs)
/** @brief Number of tasks */
LOG_ADD(LOG_UINT8, nTasks, &taskCountLog)
/** @brief Smallest free stack of all tasks [words] */
LOG_ADD(LOG_UINT16, stkMin, &stackMin)

LOG_ADD(LOG_UINT16, lTof, &slotLoad[SLOT_TOF])
LOG_ADD(LOG_UINT16, lDeck, &slotLoad[SLOT_DECK])
LOG_ADD(LOG_UINT16, lSyslink, &slotLoad[SLOT_SYSLINK])
LOG_ADD(LOG_UINT16, lCrtpRx, &slotLoad[SLOT_CRTP_RX])
LOG_ADD(LOG_UINT16, lCrtpTx, &slotLoad[SLOT_CRTP_TX])
LOG_ADD(LOG_UINT16, lStab, &slotLoad[SLOT_STAB])
LOG_ADD(LOG_UINT16, lKalman, &slotLoad[SLOT_KALMAN])
LOG_ADD(LOG_UINT16, lSensors, &slotLoad[SLOT_SENSORS])
LOG_ADD(LOG_UINT16, lApp, &slotLoad[SLOT_APP])
LOG_ADD(LOG_UINT16, lLog, &slotLoad[SLOT_LOG])
LOG_ADD(LOG_UINT16, lTmr, &slotLoad[SLOT_TMR])
LOG_ADD(LOG_UINT16, lFlow, &slotLoad[SLOT_FLOW])
/** @brief Sum of all tasks without a dedicated slot [permille] */
LOG_ADD(LOG_UINT16, lOther, &otherLoad)

LOG_ADD(LOG_UINT16, sTof, &slotStack[SLOT_TOF])
LOG_ADD(LOG_UINT16, sDeck, &slotStack[SLOT_DECK])
LOG_ADD(LOG_UINT16, sSyslink, &slotStack[SLOT_SYSLINK])
LOG_ADD(LOG_UINT16, sCrtpRx, &slotStack[SLOT_CRTP_RX])
LOG_ADD(LOG_UINT16, sApp, &slotStack[SLOT_APP])
LOG_ADD(LOG_UINT16, sTmr, &slotStack[SLOT_TMR])
LOG_GROUP_STOP(sysload)
