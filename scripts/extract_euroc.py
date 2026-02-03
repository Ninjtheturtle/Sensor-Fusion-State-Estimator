"""
extract_euroc.py — Extract only the two EuRoC CSVs from the nested machine_hall.zip.

Uses mmap + a seekable wrapper so the inner 1.5 GB MH_01_easy.zip is NEVER
written to disk; only the two tiny CSV files (~2 MB total) are extracted.
"""

import zipfile, mmap, io, os, sys, struct

OUTER_ZIP  = r"C:\Users\nengj\Downloads\machine_hall.zip"
INNER_NAME = "machine_hall/MH_01_easy/MH_01_easy.zip"
TARGETS = [
    ("mav0/imu0/data.csv",
     r"C:\Users\nengj\OneDrive\Desktop\Sensor Fusion State Estimator\data\MH_01_easy\imu0\data.csv"),
    ("mav0/state_groundtruth_estimate0/data.csv",
     r"C:\Users\nengj\OneDrive\Desktop\Sensor Fusion State Estimator\data\MH_01_easy\state_groundtruth_estimate0\data.csv"),
]


class _OffsetView:
    """Seekable, read-only file-like view of a slice of an mmap."""
    def __init__(self, mm: mmap.mmap, start: int, size: int):
        self._mm   = mm
        self._start = start
        self._size  = size
        self._pos   = 0

    def read(self, n: int = -1) -> bytes:
        if n < 0:
            n = self._size - self._pos
        n = min(n, self._size - self._pos)
        if n <= 0:
            return b""
        data = self._mm[self._start + self._pos : self._start + self._pos + n]
        self._pos += len(data)
        return data

    def seek(self, pos: int, whence: int = 0) -> int:
        if   whence == 0: self._pos = pos
        elif whence == 1: self._pos += pos
        elif whence == 2: self._pos = self._size + pos
        self._pos = max(0, min(self._pos, self._size))
        return self._pos

    def tell(self) -> int:
        return self._pos

    def seekable(self) -> bool: return True
    def readable(self) -> bool: return True
    def writable(self) -> bool: return False


def _local_header_size(mm: mmap.mmap, header_offset: int) -> int:
    """Return the size of the ZIP local file header at header_offset."""
    # Local file header signature: 0x04034b50
    sig = mm[header_offset:header_offset+4]
    if sig != b'PK\x03\x04':
        raise ValueError(f"Bad local header signature at offset {header_offset}: {sig!r}")
    fname_len  = struct.unpack_from("<H", mm, header_offset + 26)[0]
    extra_len  = struct.unpack_from("<H", mm, header_offset + 28)[0]
    return 30 + fname_len + extra_len


def main():
    print(f"Opening outer ZIP: {OUTER_ZIP}")
    if not os.path.exists(OUTER_ZIP):
        sys.exit(f"ERROR: file not found: {OUTER_ZIP}")

    with open(OUTER_ZIP, "rb") as f:
        outer = zipfile.ZipFile(f)
        info  = outer.getinfo(INNER_NAME)

        print(f"Found inner ZIP: {INNER_NAME}")
        print(f"  Compressed size : {info.compress_size:,} bytes")
        print(f"  File size       : {info.file_size:,} bytes")
        print(f"  Compression     : {info.compress_type}  (0 = STORE)")

        print(f"\nDecompressing inner ZIP into memory (~{info.file_size//1024//1024} MB)...")
        print("This may take 30-60 seconds...")
        with outer.open(INNER_NAME) as inner_stream:
            inner_bytes = io.BytesIO(inner_stream.read())
        print("Decompressed. Parsing inner ZIP...")

        inner = zipfile.ZipFile(inner_bytes)

        for inner_path, dest_path in TARGETS:
            os.makedirs(os.path.dirname(dest_path), exist_ok=True)
            print(f"\nExtracting: {inner_path}")
            with inner.open(inner_path) as src, open(dest_path, "wb") as dst:
                dst.write(src.read())
            size = os.path.getsize(dest_path)
            print(f"  -> {dest_path}  ({size:,} bytes)")

    print("\nDone! Both CSV files extracted successfully.")


if __name__ == "__main__":
    main()
