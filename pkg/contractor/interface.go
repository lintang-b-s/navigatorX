package contractor

import "github.com/lintang-b-s/navigatorx/pkg/storage/disk"

type BufferPoolManager interface {
	UnpinPage(blockID disk.BlockID, isDirty bool) bool
	FetchPage(blockID disk.BlockID) (*disk.Page, error)
}
