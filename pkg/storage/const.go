package storage

const (
	MAX_BUFFER_POOL_SIZE_IN_MB = 200
	MAX_PAGE_SIZE              = 16384
	MAX_BUFFER_POOL_SIZE       = MAX_BUFFER_POOL_SIZE_IN_MB * 1024 * 1024 / MAX_PAGE_SIZE

	DB_DIR          = "navigatorx-graphdb"
	GRAPH_FILE_NAME = "graph.index"
	LOG_FILE_NAME   = "navigatorx-graph.log"
)
