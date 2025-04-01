package buffer

import (
	"fmt"

	"github.com/lintang-b-s/navigatorx/pkg/storage"
	"github.com/lintang-b-s/navigatorx/pkg/storage/disk"
)

// https://15445.courses.cs.cmu.edu/spring2023/slides/06-bufferpool.pdf

type BufferPoolManager struct {
	bufferPool   []*Buffer // pages dari disk yang sementara disimpan di buffer.
	numAvailable int       // jumlah buffer yang tersedia.
	poolSize     int
	bufferTable  map[disk.BlockID]int // mapping antara page blockID dengan frameID/buffer index. {blockID: frameID}
	freeList     []int                // list frame yang tidak hold any page data.
	replacer     *LRUReplacer         // LRU replacer buat evict least recently used page dari buffer pool.
	// latch        *sync.Mutex
	nextBlockID int
}

// NewBufferPoolManager. initialize buffer pool manager.
func NewBufferPoolManager(numBuffers int, diskManager DiskManager,
	logManager LogManager) *BufferPoolManager {
	bufferPool := make([]*Buffer, numBuffers)
	for i := 0; i < numBuffers; i++ {
		bufferPool[i] = NewBuffer(diskManager, logManager)
	}

	fl := make([]int, numBuffers)
	for i := 0; i < numBuffers; i++ {
		fl[i] = i //
	}

	return &BufferPoolManager{bufferPool: bufferPool, numAvailable: numBuffers,
		poolSize: numBuffers, bufferTable: make(map[disk.BlockID]int), freeList: fl, replacer: NewLRUReplacer(numBuffers), nextBlockID: 0}
}

func (bpm *BufferPoolManager) getBufferAvailable() int {
	return bpm.numAvailable
}

// flushAll. flush semua buffer yang terkait dengan transactionNum.
func (bpm *BufferPoolManager) flushAll(transactionNum int) {
	for _, buffer := range bpm.bufferPool {
		if buffer.getTransactionNum() == transactionNum {
			buffer.flush()
			buffer.setDirty(false)
		}
	}
}

// UnpinPage. unpin page/buffer dengan blockID. page yang diunpin akan di evict dari buffer pool & write ke disk jika dirty page.
func (bpm *BufferPoolManager) UnpinPage(blockID disk.BlockID, isDirty bool) bool {

	if _, ok := bpm.bufferTable[blockID]; !ok {
		// not in buffer pool
		return true
	}

	frameID := bpm.bufferTable[blockID]

	page := bpm.bufferPool[frameID]
	pinCount := page.getPinCount()

	if isDirty {
		page.setDirty(true)
	}

	if pinCount <= 0 {
		// already unpinned
		return false
	}

	page.decrementPin() // decrement pin count

	if page.getPinCount() == 0 {
		// kalau pinCount = 0, unpin di replacer
		bpm.replacer.Unpin(frameID)
	}

	return true
}

/*
FetchPage. fetch page dengan block id dari buffer pool. kalau page tidak ada di buffer pool, coba read dari disk.
& put page di buffer pool. kalau page belum ada di buffer pool, ambil frameID dari freelist or dari  evict least recently used page dari buffer pool. dan replace buffer least recently used di buffer pool dengan page blockID.
*/
func (bpm *BufferPoolManager) FetchPage(blockID disk.BlockID) (*disk.Page, error) {

	item, ok := bpm.bufferTable[blockID]
	var frameID int

	if ok {
		// kalau buffer sudah ada di buffer pool
		frameID = item
		buffer := bpm.bufferPool[frameID] // get buffer from buffer pool

		buffer.incrementPin()     // incremeen pin +1, biar thread lain tahuu kalo buffer ini lagi dipake
		bpm.replacer.Pin(frameID) // remove from LRU, biar gak di evict dari buffer pool

		return buffer.getContents(), nil // return buffer
	}

	// kalau page/buffer belum ada di buffer pool,

	if len(bpm.freeList) != 0 {
		// ambil frame dari freeList, kalau freeList tidak kosong
		frameID = bpm.freeList[0]
		bpm.freeList = bpm.freeList[1:]
	} else {
		// kalau freelist kosong, evict buffer dari buffer pool
		if !bpm.replacer.Victim(&frameID) {
			// frameID berisi least recently used buffer/page
			return nil, fmt.Errorf("no available frame")
		}
	}

	replacedBuffer := bpm.bufferPool[frameID] // least recently used buffer/page

	pageBlockID := replacedBuffer.getBlockID()
	delete(bpm.bufferTable, pageBlockID)

	bpm.bufferTable[blockID] = frameID // put blockID ke pageTable

	err := replacedBuffer.assignToBlock(blockID) // flush buffer sebelumnya & assign buffer ke page yang baru & set pin = 0
	if err != nil {
		return nil, fmt.Errorf("failed to assign buffer to block %w", err)
	}
	replacedBuffer.incrementPin()

	bpm.replacer.Pin(frameID) // remove from LRU, biar gak di evict dari buffer pool
	return replacedBuffer.getContents(), nil
}

// PinPage. pin page dengan block id & put page di buffer pool. buffer/page yang di pin tidak akan dihapus dari buffer pool.
func (bpm *BufferPoolManager) PinPage(blockID disk.BlockID) (*Buffer, error) {

	allPinned := true
	for i := 0; i < bpm.poolSize; i++ {
		// find unpinned page
		if bpm.bufferPool[i].getPinCount() <= 0 {
			allPinned = false
			break
		}
	}

	if allPinned {
		// semua page pinned/used oleh thread lain,return nil
		return nil, fmt.Errorf("all pages are pinned")
	}

	var frameID int = 0

	if len(bpm.freeList) != 0 {
		// ambil dari freelist jika freelist masih ada
		frameID = bpm.freeList[0]
		bpm.freeList = bpm.freeList[1:]
	} else {
		// ambil frameID dari evicted buffer di lru replacer
		if !bpm.replacer.Victim(&frameID) {
			return nil, fmt.Errorf("no available frame")
		}

		bpm.bufferPool[frameID].ResetMemory()
		delete(bpm.bufferTable, bpm.bufferPool[frameID].getBlockID())
	}

	replacedBuffer := bpm.bufferPool[frameID]

	replacedBuffer.assignToBlock(blockID) // flush buffer sebelumnya & assign buffer ke page yang baru & set pin = 0
	replacedBuffer.incrementPin()         // incerment pin jadi 1

	bpm.bufferTable[blockID] = frameID
	bpm.bufferPool[frameID] = replacedBuffer

	bpm.replacer.Pin(frameID) // pin frameID biar tidka di evict dari buffer pool

	return replacedBuffer, nil
}

/*
NewPage. Allocates a new page on disk. dan put new buffer/page ke buffer pool.
,frameID baru di ambil dari freelist or  dari evict least recently used page dari buffer pool. dan replace buffer least recently used di buffer pool dengan page blockID.
*/
func (bpm *BufferPoolManager) NewPage(blockID *disk.BlockID) (*disk.Page, error) {

	allPinned := true
	for i := 0; i < bpm.poolSize; i++ {
		// find unpinned page
		if bpm.bufferPool[i].getPinCount() <= 0 {
			allPinned = false
			break
		}
	}

	if allPinned {
		return nil, fmt.Errorf("all pages are pinned")
	}

	var frameID int

	if len(bpm.freeList) != 0 {
		// ambil dari freelist jika freelist masih ada
		frameID = bpm.freeList[0]
		bpm.freeList = bpm.freeList[1:]
	} else {
		// ambil frameID dari evicted buffer di lru replacer
		if !bpm.replacer.Victim(&frameID) {
			return nil, fmt.Errorf("no available frame")
		}

		bpm.bufferPool[frameID].ResetMemory()
		delete(bpm.bufferTable, bpm.bufferPool[frameID].getBlockID())
	}

	replacedBuffer := bpm.bufferPool[frameID]

	newPage := disk.NewPage(storage.MAX_PAGE_SIZE)                       //create new page
	*blockID = disk.NewBlockID(storage.GRAPH_FILE_NAME, bpm.nextBlockID) // create new blockID
	bpm.nextBlockID++

	replacedBuffer.assignToBlock(*blockID) // flush buffer sebelumnya & assign buffer ke page yang baru & set pin = 0
	replacedBuffer.incrementPin()          // incerment pin jadi 1

	bpm.bufferTable[*blockID] = frameID
	bpm.bufferPool[frameID] = replacedBuffer

	bpm.replacer.Pin(frameID) // pin frameID biar tidka di evict dari buffer pool

	return newPage, nil
}

// DeletePage. Removes a page from the database, both on disk and in memory.
func (bpm *BufferPoolManager) DeletePage(blockID disk.BlockID) bool {

	frameID, ok := bpm.bufferTable[blockID]
	if !ok {
		// page tidak ada di buffer pool
		return true
	}

	if bpm.bufferPool[frameID].getPinCount() > 0 {
		// page masih di pin
		return false
	}

	deletedPage := bpm.bufferPool[frameID]

	if deletedPage.getIsDirty() {
		// kalau page yang di evict dari buffer pool adalah dirty, flush page tsb
		deletedPage.flush()
		deletedPage.setDirty(false)
	}

	delete(bpm.bufferTable, blockID)

	bpm.replacer.Remove(frameID)
	deletedPage.ResetMemory()

	bpm.freeList = append(bpm.freeList, frameID)
	return true
}
