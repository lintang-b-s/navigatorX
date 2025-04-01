package disk

import (
	"os"
)

type DiskManager struct {
	dbDir     string
	blockSize int
	isNew     bool
	openFiles map[string]*os.File
}

func NewDiskManager(dbDir string, blockSize int) *DiskManager {
	_, err := os.Stat(dbDir)
	if os.IsNotExist(err) {
		os.Mkdir(dbDir, 0755)
	}
	

	return &DiskManager{
		dbDir:     dbDir,
		blockSize: blockSize,
		isNew:     false,
		openFiles: make(map[string]*os.File),
	}
}

// Read. membaca satu block page dari disk.
func (dm *DiskManager) Read(blockID BlockID, page *Page) error {
	filename := dm.dbDir + "/" + blockID.GetFilename()
	f, err := dm.getFile(filename) // open file dengan nama filename
	if err != nil {
		return err
	}
	// Seek ke posisi blockID * blockSize
	_, err = f.Seek(int64(blockID.GetBlockNum()*dm.blockSize), 0)
	if err != nil {
		return err
	}
	_, err = f.Read(page.Contents()) // read byte array  dari file ke page (jumlah bytes yang diread sama dengan max_block_size dari page)
	if err != nil {
		return err
	}
	return nil
}

// Write. menulis satu block page ke disk.
func (dm *DiskManager) Write(blockID BlockID, page *Page) error {
	filename := dm.dbDir + "/" + blockID.GetFilename()
	f, err := dm.getFile(filename)
	if err != nil {
		return err
	}

	_, err = f.Seek(int64(blockID.GetBlockNum()*dm.blockSize), 0) // write pada offset blockID * blockSize
	if err != nil {
		return err
	}

	_, err = f.Write(page.Contents())
	if err != nil {
		return err
	}

	return nil
}

// Append. menambahkan satu block page kosong (ukuran sama dengan max_block_size) ke disk.
func (dm *DiskManager) Append(fileName string) (BlockID, error) {
	newBlockNum, err := dm.BlockLength(fileName) // get  blockID baru pada file
	if err != nil {
		return BlockID{}, err
	}

	newBlock := NewBlockID(fileName, newBlockNum)
	fileName = dm.dbDir + "/" + newBlock.GetFilename()

	b := make([]byte, dm.blockSize) // buat block kosong dengan ukuran blockSize
	f, err := dm.getFile(fileName)
	if err != nil {
		return BlockID{}, err
	}
	_, err = f.Seek(int64(newBlock.GetBlockNum()*dm.blockSize), 0)
	if err != nil {
		return BlockID{}, err
	}
	_, err = f.Write(b) // append block kosong ke file
	if err != nil {
		return BlockID{}, err
	}

	return newBlock, nil
}

// blockLength. return jumlah block page pada file.
func (dm *DiskManager) BlockLength(fileName string) (int, error) {
	f, err := dm.getFile(fileName)
	if err != nil {
		return 0, err
	}
	fi, err := f.Stat()
	if err != nil {
		return 0, err
	}
	return int(fi.Size() / int64(dm.blockSize)), nil
}

// getFile. get opened file dengan nama filename. jika file belum ada, maka file akan dibuat.
func (dm *DiskManager) getFile(filename string) (*os.File, error) {
	file, exists := dm.openFiles[filename]
	var err error
	if !exists {
		file, err = os.OpenFile(filename, os.O_RDWR|os.O_CREATE|os.O_SYNC, 0644)
		if err != nil {
			return nil, err
		}
	}

	dm.openFiles[filename] = file
	return file, nil
}

func (dm *DiskManager) BlockSize() int {
	return dm.blockSize
}

func (dm *DiskManager) IsNew() bool {
	return dm.isNew
}

func (dm *DiskManager) GetDBDir() string {
	return dm.dbDir
}
