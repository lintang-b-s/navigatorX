package disk

// BlockID. menyimpan informasi block data (page) disimpan di file mana dan di blockNum berapa di file tsb.  buat read page tinggal pakai blockID * blockSIze.
type BlockID struct {
	filename string
	blockNum int
}

func NewBlockID(filename string, blockNum int) BlockID {

	return BlockID{
		filename: filename,
		blockNum: blockNum,
	}
}

func (b *BlockID) GetFilename() string {
	return b.filename
}

func (b *BlockID) GetBlockNum() int {
	return b.blockNum
}
