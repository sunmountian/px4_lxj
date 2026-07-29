/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * Fixed-capacity chronological ring buffer.
 *
 * push() overwrites the oldest item when full and returns false so the caller
 * can expose the loss as an explicit data-quality condition.
 */
template<typename T, size_t Capacity>
class EventBuffer
{
	static_assert(Capacity > 0, "EventBuffer capacity must be positive");

public:
	bool push(const T &value)
	{
		bool retained_all = true;

		if (_size == Capacity) {
			_head = (_head + 1) % Capacity;
			--_size;
			retained_all = false;
		}

		_data[(_head + _size) % Capacity] = value;
		++_size;
		return retained_all;
	}

	void pop_front()
	{
		if (_size > 0) {
			_head = (_head + 1) % Capacity;
			--_size;
		}
	}

	void clear()
	{
		_head = 0;
		_size = 0;
	}

	size_t size() const { return _size; }
	constexpr size_t capacity() const { return Capacity; }
	bool empty() const { return _size == 0; }

	T &operator[](size_t index) { return _data[(_head + index) % Capacity]; }
	const T &operator[](size_t index) const { return _data[(_head + index) % Capacity]; }

	T &front() { return (*this)[0]; }
	const T &front() const { return (*this)[0]; }
	T &back() { return (*this)[_size - 1]; }
	const T &back() const { return (*this)[_size - 1]; }

	template<typename TimestampAccessor>
	void discard_before(uint64_t timestamp, TimestampAccessor timestamp_of)
	{
		while (!empty() && timestamp_of(front()) < timestamp) {
			pop_front();
		}
	}

private:
	T _data[Capacity] {};
	size_t _head{0};
	size_t _size{0};
};
