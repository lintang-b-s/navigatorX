package buffer

import "github.com/lintang-b-s/navigatorx/pkg/storage/disk"

type DiskManager interface {
	Read(blockID disk.BlockID, page *disk.Page) error
	Write(blockID disk.BlockID, page *disk.Page) error
	Append(fileName string) (disk.BlockID, error)
	BlockLength(fileName string) (int, error)
	BlockSize() int
}

type LogManager interface {
	Flush(lsn int) error
	Flush2() error
}

// Buffer . menyimpan page yang diambil dari disk  ke memori selama status nya masih pinned (pins > 0). jika di unpin (pins = 0) maka page akan dijadwalkan untuk diremove dari buffer pool & diwrite ke disk.
type Buffer struct {
	diskManager    DiskManager
	logManager     LogManager
	contents       *disk.Page   // page yang disimpan di buffer.
	blockID        disk.BlockID // blockID dari page. (buat nentuin offset pas write data page ke file)
	pins           int
	transactionNum int
	lsn            int

	isDirty bool // dirty flag buat nandain kalo page diupdate (isDirty = true -> harus diwrite ke disk sebelum di remove dari buffer pool)
}

func NewBuffer(diskManager DiskManager, logManager LogManager) *Buffer {
	buf := &Buffer{
		diskManager:    diskManager,
		logManager:     logManager,
		blockID:        disk.BlockID{},
		pins:           0,
		transactionNum: -1,
		lsn:            -1,
	}
	buf.contents = disk.NewPage(diskManager.BlockSize())
	return buf
}

// getContents. return page contents dari buffer
func (buf *Buffer) getContents() *disk.Page {
	return buf.contents
}

// getBlockID. 	return page blockID  dari buffer
func (buf *Buffer) getBlockID() disk.BlockID {
	return buf.blockID
}

func (buf *Buffer) isPinned() bool {
	return buf.pins > 0
}

func (buf *Buffer) getTransactionNum() int {
	return buf.transactionNum
}

// assignToBlock. read block/page (blockID) ke  buffer.contents
func (buf *Buffer) assignToBlock(blockID disk.BlockID) error {
	err := buf.flush() // flush log record dan data buffer yang sebelumnya
	if err != nil {
		return err
	}
	buf.blockID = blockID
	err = buf.diskManager.Read(blockID, buf.contents) // read block dari disk ke buf.contents
	if err != nil {
		return err
	}
	buf.pins = 0 // reset pins
	return nil
}

// flush. write data buffer & log record ke disk jika transactionNum >= 0
func (buf *Buffer) flush() error {
	if buf.transactionNum >= 0 {
		err := buf.logManager.Flush(buf.lsn)
		if err != nil {
			return err
		}
		err = buf.diskManager.Write(buf.blockID, buf.contents)
		if err != nil {
			return err
		}
		buf.transactionNum = -1
	}
	return nil
}

// incrementPin. increment pin count
func (buf *Buffer) incrementPin() {
	buf.pins++
}

// getPinCount. return pin count
func (buf *Buffer) getPinCount() int {
	return buf.pins
}

// decrementPin. decrement pin count
func (buf *Buffer) decrementPin() {
	buf.pins--
}

// setDirty. set dirty flag
func (buf *Buffer) setDirty(isDirty bool) {
	buf.isDirty = isDirty
}

// getIsDirty. return dirty flag
func (buf *Buffer) getIsDirty() bool {
	return buf.isDirty
}

// ResetMemory. reset buffer contents jadi byte array dengan capacity 0.
func (buf *Buffer) ResetMemory() {
	buf.contents = disk.NewPage(0)
}

func (buf *Buffer) getContentsSize() *disk.Page {
	return buf.contents
}

func (buf *Buffer) setModified(transactionNum int, lsn int) {
	buf.transactionNum = transactionNum
	if lsn >= 0 {
		buf.lsn = lsn
	}
	buf.isDirty = true
}
