/*
 * Internet_transmit.cpp
 * Resident mode: enter the displayed WAV file number to send it over UDP.
 *
 * [Dev-C++ build notes]
 * The ws2_32 library must be linked.
 * Menu: Tools -> Compiler Options -> add -lws2_32 under "Add the following commands when calling the compiler"
 * Command line: g++ -O2 -Wall -Wextra -o Internet_transmit.exe Internet_transmit.cpp -lws2_32
 *
 * [FPGA top-level configuration notes]
 * The FPGA top-level tx_dst_ip and tx_dst_port must be configured strictly as the PC IP (192.168.240.2)
 * and port (50000), otherwise the PC will not receive any ACK.
 * The FPGA accepts only one file per reset; if status=3 is received, reset the FPGA.
 */

#define NOMINMAX
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
/* Needed by the inlined AV25 media transport (previously pulled in by media25.h). */
#include <string>
#include <vector>
#include <algorithm>

static const char *PC_IP = "192.168.240.2";
static const char *FPGA_IP = "192.168.240.1";
static const unsigned short PC_PORT = 50000, FPGA_PORT = 1;

typedef struct {
    char name[MAX_PATH];
} WavEntry;

static int data_folder_path(char *path, size_t capacity) {
    const char *profile = getenv("USERPROFILE");
    const char *drive = getenv("HOMEDRIVE");
    const char *home = getenv("HOMEPATH");
    int written;
    if (profile && *profile)
        written = snprintf(path, capacity, "%s\\Desktop\\DATA_FOLDER", profile);
    else if (drive && home)
        written = snprintf(path, capacity, "%s%s\\Desktop\\DATA_FOLDER", drive, home);
    else
        return 0;
    return written >= 0 && (size_t)written < capacity;
}

static int wav_name_compare(const void *left, const void *right) {
    const WavEntry *a = (const WavEntry *)left;
    const WavEntry *b = (const WavEntry *)right;
    int order = _stricmp(a->name, b->name);
    return order ? order : strcmp(a->name, b->name);
}

/* Enumerate regular WAV files and give the displayed numbers a stable order. */
static int list_wav_files(const char *folder, WavEntry **entries, size_t *count) {
    char pattern[MAX_PATH];
    WIN32_FIND_DATAA found;
    HANDLE handle;
    size_t capacity = 0;
    int written = snprintf(pattern, sizeof(pattern), "%s\\*.*", folder);
    *entries = NULL;
    *count = 0;
    if (written < 0 || (size_t)written >= sizeof(pattern)) return 0;
    handle = FindFirstFileA(pattern, &found);
    if (handle == INVALID_HANDLE_VALUE) {
        if (GetLastError() == ERROR_FILE_NOT_FOUND) return 1;
        return 0;
    }
    do {
        const char* ext=strrchr(found.cFileName,'.');
        if (!(found.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && ext &&
            (_stricmp(ext,".wav")==0 || _stricmp(ext,".mp4")==0)) {
            if (*count == capacity) {
                size_t new_capacity = capacity ? capacity * 2 : 16;
                WavEntry *grown = (WavEntry *)realloc(*entries, new_capacity * sizeof(WavEntry));
                if (!grown) {
                    free(*entries);
                    *entries = NULL;
                    *count = 0;
                    FindClose(handle);
                    return 0;
                }
                *entries = grown;
                capacity = new_capacity;
            }
            strncpy((*entries)[*count].name, found.cFileName, MAX_PATH - 1);
            (*entries)[*count].name[MAX_PATH - 1] = '\0';
            ++*count;
        }
    } while (FindNextFileA(handle, &found));
    FindClose(handle);
    qsort(*entries, *count, sizeof(WavEntry), wav_name_compare);
    return 1;
}


/* ==================== CRC32 implementation ==================== */
static uint32_t crc32_table[256];
static int crc32_ready = 0;

static void init_crc32_table(void) {
    uint32_t i;
    int j;
    for (i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
        crc32_table[i] = crc;
    }
    crc32_ready = 1;
}

static uint32_t calc_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    size_t i;
    if (!crc32_ready) init_crc32_table();
    for (i = 0; i < len; i++) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}

/* ==================== big-endian read/write helpers ==================== */
static void write_u32_be(uint8_t *buf, uint32_t val) {
    buf[0] = (val >> 24) & 0xFF;
    buf[1] = (val >> 16) & 0xFF;
    buf[2] = (val >> 8) & 0xFF;
    buf[3] = val & 0xFF;
}

static uint32_t read_u32_be(const uint8_t *buf) {
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

/* 1=advance exactly one sent packet; 2=retry; 0=ignore; -1=fatal status.
 * This helper is shared with the offline protocol regression test. */
static int validate_wack(const uint8_t *buf, int len, uint32_t session,
                         uint32_t offset, uint32_t payload, uint32_t *status) {
    if (len != 20 || memcmp(buf, "WACK", 4) ||
        read_u32_be(buf + 4) != session ||
        calc_crc32(buf, 16) != read_u32_be(buf + 16)) return 0;
    *status = read_u32_be(buf + 12);
    uint32_t next = read_u32_be(buf + 8);
    if (*status > 2) return -1;
    if ((*status == 0 || *status == 1) && next == offset + payload) return 1;
    if (*status == 2 && next == offset) return 2;
    return 0;
}

/* ==================== one complete transmission ==================== */
/* Return value: 1 = whole file sent successfully; 0 = failure or no file found */
static int do_transmit(const char *selected_name) {
    /* All variable declarations are placed first, so goto never skips initialization */
    char desktop_path[MAX_PATH];
    char full_wav_path[MAX_PATH];
    char wav_file_name[MAX_PATH];
    int transmit_success;
    FILE *fp;
    SOCKET sockfd;
    WSADATA wsaData;
    BOOL opt;
    struct sockaddr_in pc_addr;
    struct sockaddr_in fpga_addr;
    DWORD timeout;
    uint32_t file_size;
    uint32_t session;
    uint32_t offset;
    uint8_t pkt_buf[1044];
    uint8_t ack_buf[2048];
    uint32_t payload_len;
    size_t read_len;
    uint32_t crc;
    int pkt_len;
    int retries;
    int ack_ok;
    int sent;
    int recvd;
    int err;
    DWORD wait_start, elapsed;
    struct sockaddr_in ack_peer;
    int peer_size, verdict;
    __int64 file_length;
    uint32_t next_offset;
    uint32_t status;

    /* Initialize all variables */
    memset(desktop_path, 0, sizeof(desktop_path));
    memset(full_wav_path, 0, sizeof(full_wav_path));
    memset(wav_file_name, 0, sizeof(wav_file_name));
    transmit_success = 0;
    fp = NULL;
    sockfd = INVALID_SOCKET;
    memset(&wsaData, 0, sizeof(wsaData));
    opt = TRUE;
    memset(&pc_addr, 0, sizeof(pc_addr));
    memset(&fpga_addr, 0, sizeof(fpga_addr));
    timeout = 1000;
    file_size = 0;
    session = 0;
    offset = 0;
    memset(pkt_buf, 0, sizeof(pkt_buf));
    memset(ack_buf, 0, sizeof(ack_buf));
    payload_len = 0;
    read_len = 0;
    crc = 0;
    pkt_len = 0;
    retries = 0;
    ack_ok = 0;
    sent = 0;
    recvd = 0;
    err = 0;
    next_offset = 0;
    status = 0;

    /* 1. Resolve the folder and use the name selected from the menu. */
    if (!data_folder_path(desktop_path, sizeof(desktop_path))) {
        printf("desktop path not resolved (USERPROFILE/HOMEDRIVE/HOMEPATH empty)\n");
        return 0;
    }
    strncpy(wav_file_name, selected_name, MAX_PATH - 1);
    wav_file_name[MAX_PATH - 1] = '\0';
    if (snprintf(full_wav_path, sizeof(full_wav_path), "%s\\%s",
                 desktop_path, wav_file_name) >= (int)sizeof(full_wav_path)) {
        printf("WAV path is too long\n");
        return 0;
    }

    /* 3. Initialize Winsock */
    if ((err = WSAStartup(MAKEWORD(2, 2), &wsaData)) != 0) {
        printf("WSAStartup failed, error %d\n", err);
        printf("Not transmit success\n");
        return 0;
    }

    /* 4. Create the UDP socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == INVALID_SOCKET) {
        err = WSAGetLastError();
        printf("socket failed, WSA error %d\n", err);
        printf("Not transmit success\n");
        goto cleanup;
    }

    /* Set SO_EXCLUSIVEADDRUSE */
    if (setsockopt(sockfd, SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
                   (const char*)&opt, sizeof(opt)) == SOCKET_ERROR) {
        err = WSAGetLastError();
        printf("Set SO_EXCLUSIVEADDRUSE failed, WSA error %d\n", err);
        goto cleanup;
    }

    /* Bind the PC source address */
    pc_addr.sin_family = AF_INET;
    pc_addr.sin_port = htons(PC_PORT);
    pc_addr.sin_addr.s_addr = inet_addr(PC_IP);

    if (bind(sockfd, (struct sockaddr *)&pc_addr, sizeof(pc_addr)) == SOCKET_ERROR) {
        err = WSAGetLastError();
        printf("bind 192.168.240.2:50000 failed, WSA error %d\n", err);
        printf("hint: error 10049 means 192.168.240.2 is not assigned to any NIC\n");
        printf("Not transmit success\n");
        goto cleanup;
    }

    /* Set the receive timeout to 1000 ms; abort this transmission if it fails */
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,
                   (const char*)&timeout, sizeof(timeout)) == SOCKET_ERROR) {
        err = WSAGetLastError();
        printf("Set SO_RCVTIMEO failed, WSA error %d\n", err);
        printf("Not transmit success\n");
        goto cleanup;
    }

    /* Configure the FPGA target address */
    fpga_addr.sin_family = AF_INET;
    fpga_addr.sin_port = htons(FPGA_PORT);
    fpga_addr.sin_addr.s_addr = inet_addr(FPGA_IP);

    /* 5. Open the WAV file and get its size */
    fp = fopen(full_wav_path, "rb");
    if (!fp) {
        printf("cannot open WAV file: %s\n", full_wav_path);
        printf("Not transmit success\n");
        goto cleanup;
    }

    if (_fseeki64(fp, 0, SEEK_END) != 0) goto cleanup;
    file_length = _ftelli64(fp);
    if (file_length < 44 || file_length > 0xFFFFFFFFLL) {
        printf("WAV size must be 44..4294967295 bytes\n");
        goto cleanup;
    }
    file_size = (uint32_t)file_length;
    if (_fseeki64(fp, 0, SEEK_SET) != 0) goto cleanup;

    if (file_size < 44) {
        printf("file too small: %s, %u bytes\n", wav_file_name, (unsigned)file_size);
        printf("Not transmit success\n");
        goto cleanup;
    }

    /* 6. Initialize the CRC32 table and generate the session */
    if (!crc32_ready) init_crc32_table();
    session = (uint32_t)(time(NULL) ^ GetCurrentProcessId() ^ GetTickCount());

    printf("sending %s: %u bytes, %u packets, session=%u\n",
           wav_file_name, (unsigned)file_size,
           (unsigned)((file_size / 1024 + (file_size % 1024 != 0))), (unsigned)session);

    /* 7. Main send loop */
    offset = 0;

    while (offset < file_size) {
        payload_len = 1024;
        if (payload_len > file_size - offset) {
            payload_len = file_size - offset;
        }

        /* Build the UDP payload */
        memcpy(pkt_buf, "WAV1", 4);
        write_u32_be(pkt_buf + 4, session);
        write_u32_be(pkt_buf + 8, offset);
        write_u32_be(pkt_buf + 12, file_size);

        /* Read file data */
        if (_fseeki64(fp, offset, SEEK_SET) != 0) goto cleanup;
        read_len = fread(pkt_buf + 16, 1, payload_len, fp);
        if (read_len != payload_len) {
            printf("read failed: offset=%u expected=%u got=%u\n",
                   (unsigned)offset, (unsigned)payload_len, (unsigned)read_len);
            printf("Not transmit success\n");
            goto cleanup;
        }

        /* Compute and write the CRC32 */
        crc = calc_crc32(pkt_buf, 16 + payload_len);
        write_u32_be(pkt_buf + 16 + payload_len, crc);

        pkt_len = 20 + payload_len;
        retries = 0;
        ack_ok = 0;

        /* Send and timeout-retransmit logic: send each packet at most 5 times */
        while (retries < 5) {
            sent = sendto(sockfd, (const char*)pkt_buf, pkt_len, 0,
                          (struct sockaddr *)&fpga_addr, sizeof(fpga_addr));
            if (sent == SOCKET_ERROR) {
                err = WSAGetLastError();
                if (err == WSAEINTR) continue;
                printf("sendto failed, WSA error %d\n", err);
                printf("Not transmit success\n");
                goto cleanup;
            }
            if (sent != pkt_len) {
                printf("sendto returned %d, expected %d\n", sent, pkt_len);
                printf("Not transmit success\n");
                goto cleanup;
            }

            /* One absolute deadline per attempt, including malformed traffic. */
            wait_start = GetTickCount();
            while (1) {
                elapsed = GetTickCount() - wait_start;
                if (elapsed >= 1000) break;
                timeout = 1000 - elapsed;
                if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,
                    (const char*)&timeout, sizeof(timeout)) == SOCKET_ERROR) goto cleanup;
                peer_size = sizeof(ack_peer);
                recvd = recvfrom(sockfd, (char*)ack_buf, sizeof(ack_buf), 0,
                                 (struct sockaddr*)&ack_peer, &peer_size);
                if (recvd == SOCKET_ERROR) {
                    err = WSAGetLastError();
                    if (err == WSAEINTR || err == WSAEMSGSIZE) continue;
                    /* On Windows, a blocking recvfrom timeout usually returns WSAETIMEDOUT,
                       and may also return WSAEWOULDBLOCK; both are treated as a timeout */
                    if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK) {
                        printf("wack wait timeout (attempt %d of 5)\n", retries + 1);
                        break;  /* Leave the inner wait loop; the outer retries++ then resends */
                    }
                    printf("recvfrom failed, WSA error %d\n", err);
                    printf("Not transmit success\n");
                    goto cleanup;
                }

                if (ack_peer.sin_family != AF_INET ||
                    ack_peer.sin_addr.s_addr != fpga_addr.sin_addr.s_addr ||
                    ack_peer.sin_port != fpga_addr.sin_port) continue;
                verdict = validate_wack(ack_buf, recvd, session, offset, payload_len, &status);
                if (verdict == 0) continue;
                next_offset = read_u32_be(ack_buf + 8);
                printf("WACK session=%u next_offset=%u status=%u\n",
                       (unsigned)session, (unsigned)next_offset, (unsigned)status);
                if (verdict < 0) {
                    if (status == 3) printf("FPGA has another session: reset FPGA before a new file.\n");
                    else if (status == 4) printf("FPGA WAV parser error: use PCM stereo 16-bit 48000 Hz.\n");
                    else printf("Unsupported FPGA status.\n");
                    goto cleanup;
                }
                if (verdict == 2) break; /* Retry without resetting the retry budget. */
                offset = next_offset;
                ack_ok = 1;
                break;
            }

            if (ack_ok) break;
            retries++;
        }

        if (!ack_ok) {
            /* No valid WACK after 5 sends of the same packet; this transmission failed */
            printf("no valid wack after 5 attempts\n");
            printf("Not transmit success\n");
            goto cleanup;
        }

        if (offset > file_size) {
            printf("wack next_offset beyond file end (offset=%u, file_size=%u)\n",
                   (unsigned)offset, (unsigned)file_size);
            printf("Not transmit success\n");
            goto cleanup;
        }
    }

    transmit_success = 1;

cleanup:
    if (transmit_success) {
        printf("%s\n", wav_file_name);
    }

    if (fp) fclose(fp);
    if (sockfd != INVALID_SOCKET) closesocket(sockfd);
    WSACleanup();

    return transmit_success;
}

/* ==================== AV25 media transport (inlined) ====================
 * The former media25.h implementation now lives here so this translation unit
 * builds on its own. media25.h on disk is intentionally left untouched.
 *
 * 32-byte big-endian header, then video (all frames' record payloads) and audio
 * interleaved per record:
 *   field 0 magic        0x41563235  "AV25"
 *   field 1 version      1
 *   field 2 width        640
 *   field 3 height       480
 *   field 4 fps          20
 *   field 5 audio_rate   48000
 *   field 6 frame_count  1..AV_MAX_FRAMES
 *   field 7 format       1  (RGB565LE)
 *
 * One record lasts 50 ms: AV_VIDEO_BYTES of RGB565LE pixels followed by
 * AV_AUDIO_BYTES of stereo S16LE (L lo, L hi, R lo, R hi per sample frame).
 * No per-row or per-frame sub-header is added.
 */
static const uint32_t AV_MAGIC            = 0x41563235U;  /* 'A''V''2''5' */
static const uint32_t AV_VERSION          = 1;
static const uint32_t AV_FORMAT_RGB565    = 1;
static const uint32_t AV_WIDTH            = 640;
static const uint32_t AV_HEIGHT           = 480;
static const uint32_t AV_FPS              = 20;
static const uint32_t AV_SAMPLE_RATE      = 48000;
static const uint32_t AV_CHANNELS         = 2;
static const uint32_t AV_BYTES_PER_SAMPLE = 2;                                            /* S16LE */
static const uint32_t AV_VIDEO_BYTES      = AV_WIDTH * AV_HEIGHT * 2;                      /* 614400 */
static const uint32_t AV_AUDIO_FRAMES     = AV_SAMPLE_RATE / AV_FPS;                       /* 2400   */
static const uint32_t AV_AUDIO_BYTES      = AV_AUDIO_FRAMES * AV_CHANNELS * AV_BYTES_PER_SAMPLE; /* 9600 */
static const uint32_t AV_HEADER_BYTES     = 32;
static const uint32_t AV_RECORD_BYTES     = AV_VIDEO_BYTES + AV_AUDIO_BYTES;               /* 624000 */
/* 6882 frames keep 32 + frames*record inside the 32-bit session size (344.1 s). */
static const uint32_t AV_MAX_FRAMES       = (0xFFFFFFFFU - AV_HEADER_BYTES) / AV_RECORD_BYTES;

struct Media25 {
    FILE *video, *audio;
    uint32_t frames, size;
    uint64_t audio_size;
    unsigned char header[AV_HEADER_BYTES];
    std::string video_path, audio_path;
    Media25() : video(NULL), audio(NULL), frames(0), size(0), audio_size(0) {}
    ~Media25() {
        if (video) fclose(video);
        if (audio) fclose(audio);
        if (!video_path.empty()) DeleteFileA(video_path.c_str());
        if (!audio_path.empty()) DeleteFileA(audio_path.c_str());
    }
};

static std::string av_quote(const std::string& s) { return "\"" + s + "\""; }

static bool av_process(const std::string& exe, const std::string& args) {
    std::string cmd = av_quote(exe) + " " + args;
    int n = MultiByteToWideChar(CP_ACP, 0, cmd.c_str(), -1, NULL, 0);
    if (!n) return false;
    std::vector<wchar_t> wide(n);
    MultiByteToWideChar(CP_ACP, 0, cmd.c_str(), -1, &wide[0], n);
    STARTUPINFOW si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    ZeroMemory(&pi, sizeof(pi));
    si.cb = sizeof(si);
    if (!CreateProcessW(NULL, &wide[0], NULL, NULL, FALSE, CREATE_NO_WINDOW,
                        NULL, NULL, &si, &pi)) {
        printf("Cannot launch FFmpeg (Windows error %lu).\n", GetLastError());
        return false;
    }
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD code = 1;
    GetExitCodeProcess(pi.hProcess, &code);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return code == 0;
}

static bool av_temp(std::string& path) {
    char dir[MAX_PATH], name[MAX_PATH];
    if (!GetTempPathA(MAX_PATH, dir) || !GetTempFileNameA(dir, "av2", 0, name)) return false;
    path = name;
    return true;
}

static bool av_prepare(const char* input, Media25& m) {
    char exe_dir[MAX_PATH];
    if (!GetModuleFileNameA(NULL, exe_dir, MAX_PATH)) return false;
    char* slash = strrchr(exe_dir, '\\');
    if (!slash) return false;
    *slash = 0;
    std::string base = std::string(exe_dir) + "\\tools\\ffmpeg\\";
    FILE* config = fopen((base + "ffmpeg_path.txt").c_str(), "rb");
    char rel[MAX_PATH] = {};
    if (!config) {
        printf("Missing tools/ffmpeg/ffmpeg_path.txt\n");
        return false;
    }
    if (!fgets(rel, sizeof(rel), config)) {
        fclose(config);
        return false;
    }
    fclose(config);
    rel[strcspn(rel, "\r\n")] = 0;
    std::string ff = base + rel;
    if (!av_temp(m.video_path) || !av_temp(m.audio_path)) return false;

    printf("Preprocessing MP4: %ux%u, %u fps, RGB565; %u Hz stereo PCM.\n",
           AV_WIDTH, AV_HEIGHT, AV_FPS, AV_SAMPLE_RATE);

    std::string common = "-nostdin -hide_banner -loglevel error -y -i " + av_quote(input);

    /* Preserve aspect ratio with black padding; zero-based shared timeline. */
    char vf[256];
    snprintf(vf, sizeof(vf),
             "fps=%u:start_time=0,scale=%u:%u:force_original_aspect_ratio=decrease,"
             "pad=%u:%u:(ow-iw)/2:(oh-ih)/2",
             AV_FPS, AV_WIDTH, AV_HEIGHT, AV_WIDTH, AV_HEIGHT);
    if (!av_process(ff, common + " -map 0:v:0 -an -vf \"" + vf + "\" -pix_fmt rgb565le -f rawvideo " +
                            av_quote(m.video_path))) {
        printf("Video decoding failed.\n");
        return false;
    }

    /* Keep the existing audio timeline correction; ALWAYS resample to 48 kHz. */
    char af[160], arg_rate[16], arg_ch[8];
    snprintf(af, sizeof(af), "aresample=%u:async=1:first_pts=0", AV_SAMPLE_RATE);
    snprintf(arg_rate, sizeof(arg_rate), "%u", AV_SAMPLE_RATE);
    snprintf(arg_ch, sizeof(arg_ch), "%u", AV_CHANNELS);
    if (!av_process(ff, common + " -map 0:a:0 -vn -af \"" + af + "\" -ac " + arg_ch + " -ar " +
                            arg_rate + " -c:a pcm_s16le -f s16le " + av_quote(m.audio_path))) {
        printf("Audio decoding failed or this MP4 has no audio track.\n");
        return false;
    }

    m.video = fopen(m.video_path.c_str(), "rb");
    m.audio = fopen(m.audio_path.c_str(), "rb");
    if (!m.video || !m.audio) return false;
    if (_fseeki64(m.video, 0, SEEK_END) || _fseeki64(m.audio, 0, SEEK_END)) return false;
    int64_t vs = _ftelli64(m.video), as = _ftelli64(m.audio);
    if (vs <= 0 || as < 0) return false;
    if (vs % (int64_t)AV_VIDEO_BYTES) {
        printf("Decoded video is %I64d bytes, not a multiple of %u (%ux%u RGB565, %u bytes/frame).\n",
               (long long)vs, AV_VIDEO_BYTES, AV_WIDTH, AV_HEIGHT, AV_VIDEO_BYTES);
        printf("Check that FFmpeg produced exactly %ux%u rgb565le frames.\n", AV_WIDTH, AV_HEIGHT);
        return false;
    }
    uint64_t frames = (uint64_t)vs / AV_VIDEO_BYTES;
    if (frames == 0) {
        printf("Decoded video contains no frame.\n");
        return false;
    }
    if (frames > AV_MAX_FRAMES) {
        printf("Decoded %I64u frames exceed the 32-bit session limit (%u frames = %.1f s).\n",
               (unsigned long long)frames, AV_MAX_FRAMES, AV_MAX_FRAMES / (double)AV_FPS);
        printf("Shorten the clip or split it into segments; nothing is truncated silently.\n");
        return false;
    }
    uint64_t size = (uint64_t)AV_HEADER_BYTES + frames * (uint64_t)AV_RECORD_BYTES;
    if (size > 0xFFFFFFFFULL) {
        printf("Prepared size %I64u bytes does not fit the 32-bit session field.\n",
               (unsigned long long)size);
        return false;
    }
    m.frames = (uint32_t)frames;
    m.size = (uint32_t)size;
    m.audio_size = (uint64_t)as;

    memcpy(m.header, "AV25", 4);
    write_u32_be(m.header + 4, AV_VERSION);
    write_u32_be(m.header + 8, AV_WIDTH);
    write_u32_be(m.header + 12, AV_HEIGHT);
    write_u32_be(m.header + 16, AV_FPS);
    write_u32_be(m.header + 20, AV_SAMPLE_RATE);
    write_u32_be(m.header + 24, m.frames);
    write_u32_be(m.header + 28, AV_FORMAT_RGB565);

    printf("Prepared %u frames (%.2f s), %u bytes/record, %u transport bytes.\n",
           m.frames, m.frames / (double)AV_FPS, AV_RECORD_BYTES, m.size);
    return true;
}

static bool av_read(Media25& m, uint32_t off, unsigned char* dst, uint32_t len) {
    if (off > m.size || len > m.size - off) return false;
    while (len) {
        uint32_t n;
        FILE* f;
        uint64_t pos;
        if (off < AV_HEADER_BYTES) {
            n = std::min(len, AV_HEADER_BYTES - off);
            memcpy(dst, m.header + off, n);
        } else {
            uint32_t record = (off - AV_HEADER_BYTES) / AV_RECORD_BYTES;
            uint32_t within = (off - AV_HEADER_BYTES) % AV_RECORD_BYTES;
            bool video = within < AV_VIDEO_BYTES;
            if (video) {
                n = std::min(len, AV_VIDEO_BYTES - within);
                f = m.video;
                pos = (uint64_t)record * AV_VIDEO_BYTES + within;
            } else {
                within -= AV_VIDEO_BYTES;
                n = std::min(len, AV_AUDIO_BYTES - within);
                f = m.audio;
                pos = (uint64_t)record * AV_AUDIO_BYTES + within;
            }
            /* Short audio is zero padded; audio past the video tail is not sent. */
            uint32_t available = video ? n
                                : (pos < m.audio_size
                                       ? (uint32_t)std::min<uint64_t>(n, m.audio_size - pos)
                                       : 0);
            if (available && (_fseeki64(f, pos, SEEK_SET) || fread(dst, 1, available, f) != available))
                return false;
            if (available < n) memset(dst + available, 0, n - available);
        }
        dst += n;
        off += n;
        len -= n;
    }
    return true;
}

/* Video uses its own protocol; never send AV25 bytes into the WAV parser.
 * VACK status: bits 7:0 error, bits 15:8 conservative free packet credits. */
// 50 us spacing caps VID1 payload at 20.48 MB/s, above 12.48 MB/s
// playback demand, without four back-to-back gigabit bursts into the 25 MHz
// receive pipeline. No catch-up bursts after stalls. All retries are paced.
static void av_packet_pace() {
    static LARGE_INTEGER hz = {}, previous = {};
    LARGE_INTEGER now;
    if (!hz.QuadPart) QueryPerformanceFrequency(&hz);
    const LONGLONG ticks = (hz.QuadPart + 19999) / 20000;
    do { QueryPerformanceCounter(&now); }
    while (previous.QuadPart && now.QuadPart - previous.QuadPart < ticks);
    previous = now;
}

static int av_send(const char* path) {
    Media25 m;
    if (!av_prepare(path, m)) return 0;
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa)) return 0;
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    bool ok = false;
    sockaddr_in local, peer;
    ZeroMemory(&local, sizeof(local));
    ZeroMemory(&peer, sizeof(peer));
    local.sin_family = peer.sin_family = AF_INET;
    local.sin_addr.s_addr = inet_addr(PC_IP);
    local.sin_port = htons(PC_PORT);
    peer.sin_addr.s_addr = inet_addr(FPGA_IP);
    peer.sin_port = htons(FPGA_PORT);
    if (s == INVALID_SOCKET || bind(s, (sockaddr*)&local, sizeof(local)) == SOCKET_ERROR) {
        printf("Video UDP bind failed: %d\n", WSAGetLastError());
    } else {
        uint32_t session = (uint32_t)(GetTickCount() ^ time(NULL) ^ GetCurrentProcessId());
        uint32_t acked = 0, sent_end = 0, high_water = 0, credit = 1;
        uint32_t last_progress = GetTickCount(), last_probe = GetTickCount();
        uint32_t valid_acks = 0, rejected_acks = 0, last_status = 0xffffffffU;
        printf("UDP started after preprocessing; 10 s timeout means no ACK offset progress.\n");
        unsigned char packet[1044], ack[64];
        DWORD timeout = 20;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
        while (acked < m.size) {
            uint32_t in_flight = (sent_end - acked + 1023) / 1024;
            while (sent_end < m.size && in_flight < credit && in_flight < 4) {
                uint32_t n = std::min<uint32_t>(1024, m.size - sent_end);
                memcpy(packet, "VID1", 4);
                write_u32_be(packet + 4, session);
                write_u32_be(packet + 8, sent_end);
                write_u32_be(packet + 12, m.size);
                if (!av_read(m, sent_end, packet + 16, n)) goto av_done;
                write_u32_be(packet + 16 + n, calc_crc32(packet, 16 + n));
                av_packet_pace();
                if (sendto(s, (char*)packet, 20 + n, 0, (sockaddr*)&peer, sizeof(peer)) != (int)(20 + n))
                    goto av_done;
                sent_end += n;
                high_water = std::max(high_water, sent_end);
                ++in_flight;
            }
            sockaddr_in source;
            ZeroMemory(&source, sizeof(source));
            int sl = sizeof(source);
            fd_set readers; FD_ZERO(&readers); FD_SET(s, &readers);
            timeval wait = {0, 1000};
            int selected = select(0, &readers, NULL, NULL, &wait);
            if (selected == SOCKET_ERROR) break;
            int n = selected > 0 ? recvfrom(s, (char*)ack, sizeof(ack), 0,
                                          (sockaddr*)&source, &sl) : SOCKET_ERROR;
            if (!selected) WSASetLastError(WSAETIMEDOUT);
            if (n == 20 && source.sin_addr.s_addr == peer.sin_addr.s_addr &&
                source.sin_port == peer.sin_port && !memcmp(ack, "VACK", 4) &&
                read_u32_be(ack + 4) == session &&
                calc_crc32(ack, 16) == read_u32_be(ack + 16)) {
                uint32_t next = read_u32_be(ack + 8), st = read_u32_be(ack + 12);
                ++valid_acks;
                last_status = st;
                if (valid_acks <= 16)
                    printf("VACK session=%u next_offset=%u status=%u credit=%u\n",
                           session, next, st & 255, (st >> 8) & 255);
                if ((st & 255) >= 3) {
                    printf("VACK fatal status=%u\n", st & 255);
                    break;
                }
                if (next < acked || next > high_water || next > m.size ||
                    (next != m.size && next % 1024)) {
                    ++rejected_acks;
                    continue;
                }
                credit = std::min<uint32_t>((st >> 8) & 255, 4);
                if (next > acked) {
                    acked = next;
                    sent_end = std::max(sent_end, acked);
                    last_progress = GetTickCount();
                    if ((acked / 1024) % 256 == 0 || acked == m.size)
                        printf("VACK %u / %u\n", acked, m.size);
                }
                if ((st & 255) == 2) sent_end = acked;
            } else if (n >= 0) {
                ++rejected_acks;
            } else if (n == SOCKET_ERROR && WSAGetLastError() != WSAETIMEDOUT &&
                       WSAGetLastError() != WSAEWOULDBLOCK && WSAGetLastError() != WSAEMSGSIZE) {
                break;
            }
            DWORD now = GetTickCount();
            if (now - last_progress > 10000) {
                printf("No VACK progress for 10 s: acked=%u sent_end=%u high_water=%u credit=%u "
                       "valid_acks=%u rejected_acks=%u last_status=0x%08X\n",
                       acked, sent_end, high_water, credit, valid_acks, rejected_acks, last_status);
                printf(valid_acks == 0
                           ? "No matching VACK: check PC/B Ethernet, addresses and firmware.\n"
                           : credit == 0
                                 ? "Receiver credits exhausted: check A digit F/6, board link and downstream consumption.\n"
                                 : "VACK received but offset stalled: inspect status and retry/receive path.\n");
                break;
            }
            // A stopped consumer can return zero credit. Poll that condition
            // without the old 100 ms playback gaps. Missing packets use a
            // longer retry interval; neither case resets the 10 s watchdog.
            if (now - last_probe >= (credit == 0 ? 2U : 4U) &&
                now - last_progress >= (credit == 0 ? 2U : 4U)) {
                /* Replay an unacknowledged packet, or query credits with the last committed packet. */
                uint32_t off = acked < sent_end ? acked : (acked ? ((acked - 1) / 1024) * 1024 : 0);
                uint32_t bytes = std::min<uint32_t>(1024, m.size - off);
                memcpy(packet, "VID1", 4);
                write_u32_be(packet + 4, session);
                write_u32_be(packet + 8, off);
                write_u32_be(packet + 12, m.size);
                if (!av_read(m, off, packet + 16, bytes)) break;
                write_u32_be(packet + 16 + bytes, calc_crc32(packet, 16 + bytes));
                av_packet_pace();
                sendto(s, (char*)packet, 20 + bytes, 0, (sockaddr*)&peer, sizeof(peer));
                if (acked < sent_end) sent_end = acked + bytes;
                last_probe = now;
            }
        }
        ok = acked == m.size;
    }
av_done:
    if (s != INVALID_SOCKET) closesocket(s);
    WSACleanup();
    printf(ok ? "AV upload complete; final buffered audio/video may still be playing.\n"
              : "AV transmit failed.\n");
    return ok ? 1 : 0;
}

/* ==================== main: numbered resident menu ==================== */
int main(int argc,char** argv) {
    if(argc==3 && strcmp(argv[1],"--prepare-only")==0){
        init_crc32_table();Media25 media;
        return av_prepare(argv[2],media)?0:1;
    }
    char line[64];
    char folder[MAX_PATH];
    char selected[MAX_PATH];
    WavEntry *entries;
    size_t count, i, len;
    char *end;
    unsigned long choice;

    init_crc32_table();
    if (!data_folder_path(folder, sizeof(folder))) {
        printf("desktop path not resolved (USERPROFILE/HOMEDRIVE/HOMEPATH empty)\n");
        return 1;
    }
    printf("[build] NET26 PACED 2026-10-31: AV25 %ux%u @ %u fps + %u Hz stereo; matching 20 fps FPGA candidate required\n",
           AV_WIDTH, AV_HEIGHT, AV_FPS, AV_SAMPLE_RATE);
    printf("        record=%u B (%u video + %u audio, %.2f ms), max %u frames (%.1f s)\n",
           AV_RECORD_BYTES, AV_VIDEO_BYTES, AV_AUDIO_BYTES, 1000.0 / AV_FPS,
           AV_MAX_FRAMES, AV_MAX_FRAMES / (double)AV_FPS);
    printf("Media folder: %s\n", folder);

    while (1) {
        if (!list_wav_files(folder, &entries, &count)) {
            printf("Cannot list media files in %s\n", folder);
            return 1;
        }
        printf("\nWAV / MP4 files (sorted by filename):\n");
        for (i = 0; i < count; ++i)
            printf("%u. %s\n", (unsigned)(i + 1), entries[i].name);
        if (count == 0) printf("No WAV or MP4 files found.\n");
        printf("Enter file number to send, r to refresh, or q to quit: ");
        fflush(stdout);
        if (fgets(line, sizeof(line), stdin) == NULL) {
            free(entries);
            break;
        }
        len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';
        if (_stricmp(line, "q") == 0) { free(entries); break; }
        if (_stricmp(line, "r") == 0) { free(entries); continue; }
        errno = 0;
        choice = strtoul(line, &end, 10);
        if (errno || end == line || *end != '\0' || choice == 0 || choice > count) {
            printf("Invalid file number.\n");
            free(entries);
            continue;
        }
        strncpy(selected, entries[choice - 1].name, sizeof(selected));
        selected[MAX_PATH - 1] = '\0';
        free(entries);
        const char* ext=strrchr(selected,'.');
        if(ext && _stricmp(ext,".mp4")==0){
            char input[MAX_PATH];
            int written=snprintf(input,sizeof(input),"%s\\%s",folder,selected);
            if(written<0 || (size_t)written>=sizeof(input))printf("Input path too long.\n");
            else av_send(input);
        }else do_transmit(selected);
    }
    return 0;
}
