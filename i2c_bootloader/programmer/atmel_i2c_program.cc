#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sysexits.h>
#include <boost/format.hpp>
#include <cinttypes>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

constexpr uint32_t kNonVolatileMemoryPageSizeBytes = 64;
constexpr uint8_t kSamD11I2CBootLoaderWriteOkStatus = 's';
constexpr uint32_t kWriteDelayMicroseconds = 500 * 1000;

std::string argv0;

void Usage() {
  std::cerr << (boost::format("usage: %1% i2c-device i2c-address binary") %
                argv0)
            << std::endl;
  std::exit(EX_USAGE);
}

void Fail(int exit_code, const std::string &message) {
  std::cerr << argv0 << ": " << message << std::endl;
  std::exit(exit_code);
}

void ReadOrLose(int fd, void *buf, ssize_t count) {
  ssize_t rc = read(fd, buf, count);
  if (rc != count) {
    Fail(EX_IOERR,
         (boost::format("failed to read bytes, rc=%1%, errno=%2%") % rc % errno)
             .str());
  }
}

void WriteOrLose(int fd, const void *buf, ssize_t count) {
  ssize_t rc = write(fd, buf, count);
  if (rc != count) {
    Fail(EX_IOERR, (boost::format("failed to write bytes, rc=%1%, errno=%2%") %
                    rc % errno)
                       .str());
  }
}

std::vector<std::uint8_t> ReadFileOrLose(const std::string &path) {
  const int fd = open(path.c_str(), O_RDONLY);
  if (fd < 0) {
    Fail(EX_NOINPUT, (boost::format("failed to open %1%") % path).str());
  }
  struct stat stat;
  const int rc = fstat(fd, &stat);
  if (rc < 0) {
    Fail(EX_IOERR, (boost::format("failed to stat %1%") % path).str());
  }

  std::vector<std::uint8_t> bytes(stat.st_size);
  ReadOrLose(fd, &bytes[0], bytes.size());

  if (close(fd) < 0) {
    Fail(EX_IOERR, (boost::format("failed to close %1%") % path).str());
  }

  return bytes;
}

void ReadStatusOrLose(int fd) {
  char response;
  ssize_t rc = read(fd, &response, 1);
  if (rc == 0) {
    Fail(EX_IOERR, "Did not read any bytes.");
  } else if (rc < 0) {
    Fail(EX_IOERR, (boost::format("Read error, errno=%1%") % errno).str());
  }

  if (rc != 1) {
    Fail(EX_SOFTWARE, "should not happen");
  }

  if (response != kSamD11I2CBootLoaderWriteOkStatus) {
    Fail(EX_IOERR, "Did not get expected I2C response.");
  }
}

void ProgramAtmel(int fd, const std::vector<std::uint8_t> &bytes) {
  if (bytes.size() > 0xffffffff) {
    Fail(EX_DATAERR, "program is too large");
  }
  uint32_t size = bytes.size();
  const uint8_t length_bigendian[] = {static_cast<uint8_t>((size >> 24) & 0xff),
                                      static_cast<uint8_t>((size >> 16) & 0xff),
                                      static_cast<uint8_t>((size >> 8) & 0xff),
                                      static_cast<uint8_t>(size & 0xff)};

#ifdef NOISY
  std::cout << boost::format(
                   "writing length "
                   "(0x%1$02x 0x%2$02x 0x%3$02x 0x%4$02x)") %
                   static_cast<int>(length_bigendian[0]) %
                   static_cast<int>(length_bigendian[1]) %
                   static_cast<int>(length_bigendian[2]) %
                   static_cast<int>(length_bigendian[3])
            << std::endl;
#endif

  WriteOrLose(fd, length_bigendian, sizeof(length_bigendian));
  for (uint32_t i = 0; i < size; i += kNonVolatileMemoryPageSizeBytes) {
    uint32_t n_remaining = size - i;
    uint32_t n = std::min(n_remaining, kNonVolatileMemoryPageSizeBytes);

#ifdef NOISY
    std::cout << "writing " << n << " bytes" << std::endl;
#endif

    WriteOrLose(fd, &bytes[i], n);

    // It would be better to retry the following read on errno==EIO
    // (slave is busy) instead of just having an arbitrary delay.
    // However, on the rPi, retrying the read leaves something leaves
    // something in an unrecoverable bad state.
    //
    // TODO(charliehotel): investigate further; try on another
    // platform.

    usleep(kWriteDelayMicroseconds);

    uint8_t response;
    ssize_t rc = read(fd, &response, 1);
    if (rc == 0) {
      Fail(EX_IOERR, "Did not read any bytes for status.");
    } else if (rc < 0) {
      Fail(EX_IOERR,
           (boost::format("Status read error, errno=%1%") % errno).str());
    }
    if (rc != 1) {
      Fail(EX_SOFTWARE, "should not happen");
    }
    if (response != kSamD11I2CBootLoaderWriteOkStatus) {
      Fail(EX_IOERR, "Did not get expected I2C response.");
    }
  }
}

int main(int argc, char **argv) {
  argv0 = argv[0];

  if (argc != 4) {
    Usage();
  }

  const std::string device_path(argv[1]);
  const int i2c_address = std::stoi(argv[2], 0, 0);
  const std::string binary_path(argv[3]);

  if (i2c_address <= 0 || i2c_address > 0x7f) {
    Fail(EX_DATAERR,
         (boost::format("bad i2c address: %1%") % i2c_address).str());
  }

  const auto &binary = ReadFileOrLose(binary_path);

#ifdef NOISY
  std::cout << "Binary is " << binary.size() << " bytes." << std::endl;
#endif

  // TODO(anyone): Do GPIO twiddling to put the MCU in bootloader
  // mode.

  int fd = open(device_path.c_str(), O_RDWR);
  if (fd < 0) {
    Fail(EX_NOINPUT,
         (boost::format("failed to open i2c device %1%") % device_path).str());
  }

  if (ioctl(fd, I2C_SLAVE, i2c_address) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  if (ioctl(fd, I2C_RETRIES, 5) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  if (ioctl(fd, I2C_TIMEOUT, 500 /* tens of ms */) < 0) {
    Fail(EX_IOERR, "failed to set I2C address");
  }

  ProgramAtmel(fd, binary);

  if (close(fd) < 0) {
    Fail(EX_IOERR, "failed to close I2C device");
  }

  exit(0);
}
