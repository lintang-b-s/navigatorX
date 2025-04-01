package log

import (
	"github.com/lintang-b-s/navigatorx/pkg/storage/disk"
)

type DiskManager interface {
	Read(blockID disk.BlockID, page *disk.Page) error
	Write(blockID disk.BlockID, page *disk.Page) error
	Append(fileName string) (disk.BlockID, error)
	BlockLength(fileName string) (int, error)
	BlockSize() int
	GetDBDir() string
}

// buat write & read log records ke log file.
type LogManager struct {
	diskManager    DiskManager
	logFile        string
	logPage        *disk.Page
	currentBlockID disk.BlockID // current block id dari lsn
	latestLSN      int          // log sequence number : log record identifier . LSn terakhir di memori
	lastSavedLSN   int          // LSN terakhir yang sudah diwrite ke disk
}

func NewLogManager(diskManager DiskManager, logFile string) (*LogManager, error) {

	b := make([]byte, diskManager.BlockSize())
	logPage := disk.NewPageFromByteSlice(b)          // create new page for log file
	logSize, err := diskManager.BlockLength(logFile) // get jumlah block pada log file

	if err != nil {
		return &LogManager{}, err
	}

	lm := &LogManager{
		diskManager:    diskManager,
		logFile:        logFile,
		logPage:        logPage,
		currentBlockID: disk.BlockID{},
		latestLSN:      0,
		lastSavedLSN:   0,
	}

	if logSize == 0 {
		// jika jumlah block==0 (log file kosong), maka tambahkan block baru
		lm.currentBlockID, err = lm.appendNewBlock()
		if err != nil {
			return &LogManager{}, err
		}
	} else {
		// else read dari disk , read block terakhir
		lm.currentBlockID = disk.NewBlockID(logFile, logSize-1)
		err = diskManager.Read(lm.currentBlockID, logPage)
		if err != nil {
			return &LogManager{}, err
		}
	}

	return lm, nil
}

// fluflush2sh. flush logPage  ke disk, write offset == currentBlockID*blockSize
func (lm *LogManager) Flush(lsn int) error {
	if lsn > lm.lastSavedLSN {
		err := lm.Flush2()
		return err
	}
	return nil
}

// fluflush2sh. flush logPage  ke disk, write offset pada file == currentBlockID*blockSize
func (lm *LogManager) Flush2() error {
	err := lm.diskManager.Write(lm.currentBlockID, lm.logPage)
	if err != nil {
		return err
	}
	lm.lastSavedLSN = lm.latestLSN // update lastSavedLSN
	return nil
}

// appendNewBlock. menambahkan block baru kosong ke logfile. & write logPage ke disk di offset block yang baru.
func (lm *LogManager) appendNewBlock() (disk.BlockID, error) {
	block, err := lm.diskManager.Append(lm.logFile) // append block baru ke log file
	if err != nil {
		return disk.BlockID{}, err
	}

	lm.logPage.PutInt(0, int32(lm.diskManager.BlockSize())) // set blockSize pada logPage
	err = lm.diskManager.Write(block, lm.logPage)           // write logPage ke disk
	if err != nil {
		return disk.BlockID{}, err
	}
	return block, nil
}

func (lm *LogManager) GetIterator() (*LogIterator, error) {
	lm.Flush2()
	return NewLogIterator(lm.diskManager, lm.currentBlockID)
}

/*
append. append log record ke log buffer. log record ditulis dari kanan ke kiri pada log buffer per block.
pada awal buffer terdapat lokasi record yang ditulis paling terakhir.

iterate log record perblocknya dari kiri ke kanan shg urutan iterasinya dari log yang terakhir ditambahkan ke yang terdahulu.
*/
func (lm *LogManager) append(logRecord []byte) (int, error) {
	logBlockSize := lm.logPage.GetInt(0) // get blockSize dari logPage (tergantung MAX_PAGE_SIZE )
	recordSize := len(logRecord)         // get size dari logRecord
	bytesNeeded := int32(recordSize + 4)        // bytesNeeded = recordSize + 4 (4 bytes untuk menyimpan recordSize). bytesneeded untuk simpan logRecord
	var err error
	if bytesNeeded+4 > logBlockSize {
		// jika recordSize > logBlockSize,  flush block sebelumnya ke disk &  create new block.
		lm.Flush2()
		lm.currentBlockID, err = lm.appendNewBlock() // update currentBlockID ke next blockID
		if err != nil {
			return 0, err
		}
		logBlockSize = lm.logPage.GetInt(0)
	}

	recordPosition := logBlockSize - bytesNeeded // posisi record yang ditulis paling akhir

	lm.logPage.PutBytes(recordPosition, logRecord) // write logRecord ke logPage pada offset recordPosition
	lm.logPage.PutInt(0, recordPosition)           // update sisa blockSize pada logPage
	lm.latestLSN++                                 // update latestLSN
	return lm.latestLSN, nil
}
