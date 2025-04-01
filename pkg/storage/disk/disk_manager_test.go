package disk

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestReadWriteFile(t *testing.T) {

	f1 := NewDiskManager("lintangdb", 8192)
	page := NewPage(8192)
	page.PutInt(0, 1)
	page.PutInt(4, 2)
	page.PutInt(8, 3)
	page.PutString(12, "lintang")
	newBlockID := NewBlockID("test.db", 0)
	err := f1.Write(newBlockID, page)
	if err != nil {
		t.Error(err)
	}

	pageReader := NewPage(8192)
	err = f1.Read(newBlockID, pageReader)
	if err != nil {
		t.Error(err)
	}
	assert.Equal(t, 1, pageReader.GetInt(0))
	assert.Equal(t, 2, pageReader.GetInt(4))
	assert.Equal(t, 3, pageReader.GetInt(8))
	assert.Equal(t, "lintang", pageReader.GetString(12))
}
