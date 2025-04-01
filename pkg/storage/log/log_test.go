package log

import (
	"fmt"
	"os"
	"testing"

	"github.com/lintang-b-s/navigatorx/pkg/storage/disk"
	"github.com/stretchr/testify/assert"
)

func createLogMessage(name string) []byte {
	bufSize := len([]byte(name))
	buf := make([]byte, bufSize+4)
	page := disk.NewPageFromByteSlice(buf)
	page.PutString(0, name)
	return page.Contents()
}

func createLogRecordAndAppendToLogFile(t *testing.T, lm *LogManager, start, end int) {

	for i := start; i < end; i++ {
		newLogRecord := createLogMessage(fmt.Sprintf("lintang %d", i))
		lsn, err := lm.append(newLogRecord)
		if err != nil {
			t.Errorf("Error appending log record: %s  ke-%d", err, i)
		}
		assert.Equal(t, i+1, lsn)
	}
}
func printLogRecord(t *testing.T, lm *LogManager, maxLogIDx int) {
	logIterator, err := lm.GetIterator()
	if err != nil {
		t.Errorf("Error creating log iterator: %s", err)
	}

	logIDx := maxLogIDx - 1
	for log := range logIterator.IterateLog() {
		page := disk.NewPageFromByteSlice(log)

		// t.Logf("Log record: %s", page.GetString(0))
		assert.Equal(t, fmt.Sprintf("lintang %d", logIDx), page.GetString(0))
		logIDx--
	}
	if logIterator.GetError() != nil {
		t.Errorf("Error iterating log record: %s", logIterator.GetError())
	}
}

func TestLogManager(t *testing.T) {
	dm := disk.NewDiskManager("lintangdb", 8192)
	_, err := os.Stat("lintangdb")
	if err == nil {
		os.Remove("lintangdb.log")
	}
	lm, err := NewLogManager(dm, "lintangdb.log")
	if err != nil {
		t.Errorf("Error creating log manager: %s", err)
	}
	t.Run("test insert log records", func(t *testing.T) {
		createLogRecordAndAppendToLogFile(t, lm, 0, 10000)
	})

	t.Run("test iterate log records", func(t *testing.T) {
		printLogRecord(t, lm, 10000)
	})
}
