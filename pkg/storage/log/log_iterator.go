package log

import (
	"iter"

	"github.com/lintang-b-s/navigatorx/pkg/storage/disk"
)

// LogIterator. buat iterate log record yang udah ditulis di file. iteratenya darii yang terakhir ditulis ke yang terdahulu.
type LogIterator struct {
	diskManager DiskManager
	blockID     disk.BlockID
	page        *disk.Page
	currentPos  int
	blockSize   int
	err         error
}

func NewLogIterator(diskManager DiskManager, blockID disk.BlockID) (*LogIterator, error) {
	page := disk.NewPageFromByteSlice(make([]byte, diskManager.BlockSize()))
	err := diskManager.Read(blockID, page) // read blockID dari file
	if err != nil {
		return &LogIterator{}, err
	}

	lit := &LogIterator{
		diskManager: diskManager,
		blockID:     blockID,
		page:        page,
		currentPos:  0,
		blockSize:   int(page.GetInt(0)),
		err:         nil,
	}
	lit.moveToBlock(blockID) // move iterator ke blockID
	return lit, nil
}

// moveToBlock. move iterator ke blockID.
func (lit *LogIterator) moveToBlock(blockID disk.BlockID) error {
	err := lit.diskManager.Read(blockID, lit.page)
	if err != nil {
		return err
	}
	lit.blockSize = int(lit.page.GetInt(0))
	lit.currentPos = lit.blockSize
	return nil
}

/*
IterateLog. iterate next log record di dalam block dari yang terkini ke yang terdahulu. jika sudah habis, maka pindah ke block sebelumnya.

	iterate log record perblocknya dari kiri ke kanan shg urutan iterasinya dari log yang terakhir ditambahkan ke yang terdahulu.
*/
func (lit *LogIterator) IterateLog() iter.Seq[[]byte] {
	return func(yield func([]byte) bool) {
		for lit.blockID.GetBlockNum() >= 0 {

			if lit.currentPos >= lit.diskManager.BlockSize() {
				// jika sudah habis, maka pindah ke block sebelumnya.
				block := disk.NewBlockID(lit.blockID.GetFilename(), lit.blockID.GetBlockNum()-1)
				if block.GetBlockNum() < 0 {
					break
				}
				lit.blockID = block
				err := lit.moveToBlock(block) // buat  iterator.page  read block baru
				if err != nil {
					lit.err = err
					break
				}
			}

			record := lit.page.GetBytes(int32(lit.currentPos)) // get satu logRecord dari currentPos
			lit.currentPos += 4 + len(record)           // increment currentPos + 4 ( karena ada length di awal record)

			if !yield(record) {
				return
			}
		}

	}
}

func (lit *LogIterator) GetError() error {
	return lit.err
}
