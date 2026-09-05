// SPDX-License-Identifier: AGPL-3.0-only
#include "Logging.h"
#include <io.h>
#include <fcntl.h>
#include <string_view>

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    if (std::string_view(argv[1]) == "failed-close") {
        int descriptors[2];
        if (_pipe(descriptors, 4096, _O_BINARY) != 0) return 2;
        FILE* file = _fdopen(descriptors[1], "w");
        if (!file) return 2;
        LogFile = file;
        // A closed reader makes the buffered flush fail with a real I/O error.
        // Failing diagnostics must never terminate the hosting vrserver process.
        fputs("pending log", file);
        _close(descriptors[0]);
        CloseLogFile();
        return LogFile == stderr ? 0 : 1;
    }
    if (std::string_view(argv[1]) == "stderr") {
        LogFile = stderr;
        CloseLogFile(); CloseLogFile();
        return fputs("PASS: fallback stderr remains owned by host\n", stderr) >= 0 ? 0 : 1;
    }
    return 2;
}
