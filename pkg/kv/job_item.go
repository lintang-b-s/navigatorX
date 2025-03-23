package kv

type SaveWayJobItem struct {
	KeyStr string
	ValArr []kvEdge
}

type JobI interface {
	[]int32 | SaveWayJobItem
}

type Job[T JobI] struct {
	ID      int
	JobItem T
}
type JobFunc[T JobI, G any] func(job T) G
