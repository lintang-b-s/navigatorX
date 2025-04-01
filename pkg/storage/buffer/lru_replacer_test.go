package buffer

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestLRUReplacer(t *testing.T) {
	lruReplacer := NewLRUReplacer(5)

	t.Run("test lru replacer", func(t *testing.T) {
		lruReplacer.Unpin(1)
		lruReplacer.Unpin(2)
		lruReplacer.Unpin(3)
		lruReplacer.Unpin(4)
		lruReplacer.Unpin(5)

		var evictedFrameID int
		lruReplacer.Victim(&evictedFrameID)
		assert.Equal(t, 1, evictedFrameID)
		lruReplacer.Victim(&evictedFrameID)
		assert.Equal(t, 2, evictedFrameID)
		lruReplacer.Victim(&evictedFrameID)
		assert.Equal(t, 3, evictedFrameID)

		lruReplacer.Pin(4) // hapus 4 dari lru ( yang di evict selanutnya adalah 5)
		lruReplacer.Victim(&evictedFrameID)
		assert.Equal(t, 5, evictedFrameID)

		lruReplacer.Unpin(7)
		lruReplacer.Unpin(8)
		lruReplacer.Unpin(9)

		lruReplacer.Pin(5)

		lruReplacer.Victim(&evictedFrameID)
		assert.Equal(t, 7, evictedFrameID)
		lruReplacer.Victim(&evictedFrameID)
		assert.Equal(t, 8, evictedFrameID)
		lruReplacer.Victim(&evictedFrameID)
		assert.Equal(t, 9, evictedFrameID)

	})
}
