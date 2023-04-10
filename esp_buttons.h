/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2023 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ESP32 momentary button driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#ifndef __ESP_BUTTONS_H__
#define __ESP_BUTTONS_H__

#include <stdint.h>
#include <list>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ESP32 drivers */
#include "driver/gpio.h"

/* STL */
#include <vector>

#ifndef BUTTONS_TASK_SIZE
#define BUTTONS_TASK_SIZE		2048
#endif

class buttons
{
public:
	/**
	* @brief Creates a new task polling the gpio and calling the callback.
	* @param interval_ms	Poll interval in ms, default is 100ms (10Hz).
	* @param name		Polling task name.
	* @param core		Core num to pin the task,
	*			-1 if let the RTOS choose.
	* @param prio		Task priority.
	*/
	buttons(uint interval_ms = 100,
		const char *name = "Buttons",
		int core = -1,
		uint prio = 1)
	{
		lock = xSemaphoreCreateMutex();
		ESP_ERROR_CHECK(lock == nullptr);

		alive = xSemaphoreCreateBinary();
		ESP_ERROR_CHECK(alive == nullptr);

		poll_interval = interval_ms;

		if (core < 0 || core > 1)
			ESP_ERROR_CHECK(xTaskCreate(handler, name,
				BUTTONS_TASK_SIZE, this, prio,
				nullptr) != pdTRUE);
		else
			ESP_ERROR_CHECK(xTaskCreatePinnedToCore(handler, name,
				BUTTONS_TASK_SIZE, this, prio,
				nullptr, core) != pdTRUE);
	}

	/**
	 * @brief Destroy the buttons object.
	 */
	~buttons() {
		/* Call self destructor */
		is_active = false;

		/* Wait self for completion */
		xSemaphoreTake(alive, portMAX_DELAY);
		vSemaphoreDelete(alive);
	}

	/**
	 * @brief Define trigger edge.
	 */
	enum btn_trig_edge { NEGEDGE, POSEDGE };

	/**
	* @brief Add new button to list and assign the callback.
	* @param pin		Gpio pin number.
	* @param func		Callback function 'void func(void *param)'.
	* @param trig		Trigger action on positive or negative edge.
	* @param param		User parameter.
	*
	* @return 		0 on success, error code if failed.
	*/
	void add(int pin,
		void (*callback)(void *arg),
		enum btn_trig_edge trigger = NEGEDGE,
		void *param = nullptr)
	{
		struct button *btn = new button;
		ESP_ERROR_CHECK(btn == nullptr);
		btn->pin = pin;
		btn->callback = callback;
		btn->trigger = trigger;
		btn->param = param;
		btn->prev_state = trigger == POSEDGE;
		btn->index = index++;

		ESP_ERROR_CHECK(gpio_reset_pin((gpio_num_t)pin));
		ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)pin,
			GPIO_MODE_INPUT));

		/* POSEDGE ===> PULLDOWN, NEGEDGE ===> PULLUP */
		ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)pin,
			trigger == NEGEDGE ? \
				GPIO_PULLUP_ONLY : GPIO_PULLDOWN_ONLY));

		ESP_ERROR_CHECK(gpio_pullup_en((gpio_num_t)pin));

		xSemaphoreTake(lock, portMAX_DELAY);
		list.push_back(btn);
		xSemaphoreGive(lock);
	}

	/**
	 * @brief Wait for trigger.
	 *
	 * @return int		Index of button pressed.
	 */
	int wait_for_action() {
		if (waiter)
			return -1; /* already occupied */
		waiter = xTaskGetCurrentTaskHandle();
		vTaskSuspend(nullptr);
		waiter = nullptr;
		return pressed;
	}

private:
	struct button {
		int pin;
		void (*callback)(void *arg);
		enum btn_trig_edge trigger;
		void *param;
		bool prev_state;
		int index;
	};

	static void handler(void *param) {
		buttons *s = static_cast<buttons *>(param);

		while (s->is_active) {
			vTaskDelay(pdMS_TO_TICKS(s->poll_interval));

			xSemaphoreTake(s->lock, portMAX_DELAY);

			for (struct button *btn : s->list) {
				bool state =
					gpio_get_level((gpio_num_t)btn->pin);
				if (btn->prev_state != state) {
					if (state != (btn->trigger == NEGEDGE))
						call_handler(s, btn);
					btn->prev_state = state;
				}
			}

			xSemaphoreGive(s->lock);
		}

		for (struct button *btn : s->list)
			delete(btn);
		vSemaphoreDelete(s->lock);
		xSemaphoreGive(s->alive);
		vTaskDelete(nullptr);
	}

	static void call_handler(buttons *s, struct button *btn) {
		if (btn->callback)
			btn->callback(btn->param);

		/* Cheeck task is waiting for the trigger */
		if (s->waiter) {
			/* Solve race condition */
			if (eTaskGetState(s->waiter) != eSuspended)
				return;

			/* Store the pressed button index */
			s->pressed = btn->index;

			/* Wake-up the waiter */
			vTaskResume(s->waiter);
		}
	}

	std::vector<struct button *> list = { };
	uint poll_interval;
	TaskHandle_t waiter = nullptr;
	bool is_active = true;
	int index = 0;
	int pressed = -1;
	SemaphoreHandle_t lock;
	SemaphoreHandle_t alive;
};

#endif /* __ESP_BUTTONS_H__ */
