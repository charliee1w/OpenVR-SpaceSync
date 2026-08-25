// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Sound.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sound
{
	namespace
	{
		const double kVolume = 0.5;

		std::mutex mutex;
		std::condition_variable wake;
		std::deque<std::string> queue;
		std::map<std::string, std::vector<uint8_t>> cache;
		std::thread worker;
		bool running = false;
		std::string soundDir;

		std::string ExeDirectory()
		{
			char path[MAX_PATH] = {};
			DWORD n = GetModuleFileNameA(nullptr, path, MAX_PATH);
			std::string s(path, n);
			size_t slash = s.find_last_of("\\/");
			return slash == std::string::npos ? "." : s.substr(0, slash);
		}

		uint32_t ReadU32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24); }
		uint16_t ReadU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

		void ScalePcm16(std::vector<uint8_t>& wav)
		{
			if (wav.size() < 12 || std::memcmp(wav.data(), "RIFF", 4) != 0 || std::memcmp(wav.data() + 8, "WAVE", 4) != 0)
				return;
			size_t pos = 12;
			uint16_t format = 0, bits = 0;
			while (pos + 8 <= wav.size())
			{
				uint32_t size = ReadU32(wav.data() + pos + 4);
				const uint8_t* body = wav.data() + pos + 8;
				if (pos + 8 + size > wav.size())
					size = (uint32_t)(wav.size() - pos - 8);
				if (std::memcmp(wav.data() + pos, "fmt ", 4) == 0 && size >= 16)
				{
					format = ReadU16(body);
					bits = ReadU16(body + 14);
				}
				else if (std::memcmp(wav.data() + pos, "data", 4) == 0)
				{
					if (format == 1 && bits == 16)
					{
						int16_t* samples = reinterpret_cast<int16_t*>(wav.data() + pos + 8);
						size_t count = size / 2;
						for (size_t i = 0; i < count; i++)
							samples[i] = (int16_t)(samples[i] * kVolume);
					}
					return;
				}
				pos += 8 + size + (size & 1);
			}
		}

		const std::vector<uint8_t>* Load(const std::string& name)
		{
			auto it = cache.find(name);
			if (it != cache.end())
				return it->second.empty() ? nullptr : &it->second;

			std::vector<uint8_t>& buffer = cache[name];
			std::ifstream file(soundDir + "\\sound\\" + name + ".wav", std::ios::binary);
			if (file)
			{
				buffer.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
				ScalePcm16(buffer);
			}
			return buffer.empty() ? nullptr : &buffer;
		}

		void WorkerLoop()
		{
			for (;;)
			{
				std::string name;
				{
					std::unique_lock<std::mutex> lock(mutex);
					wake.wait(lock, [] { return !running || !queue.empty(); });
					if (!running)
						return;
					name = queue.front();
					queue.pop_front();
				}
				const std::vector<uint8_t>* wav = Load(name);
				if (wav)
					PlaySoundA(reinterpret_cast<LPCSTR>(wav->data()), nullptr, SND_MEMORY | SND_SYNC | SND_NODEFAULT);
			}
		}
	}

	void Init()
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (running)
			return;
		soundDir = ExeDirectory();
		running = true;
		worker = std::thread(WorkerLoop);
	}

	void Shutdown()
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running)
				return;
			running = false;
			queue.clear();
		}
		wake.notify_all();
		PlaySoundA(nullptr, nullptr, 0);
		if (worker.joinable())
			worker.join();
	}

	void Play(const char* name)
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running)
				return;
			queue.push_back(name);
		}
		wake.notify_one();
	}

	void ClearQueue()
	{
		std::lock_guard<std::mutex> lock(mutex);
		queue.clear();
	}
}
