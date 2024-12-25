package util

type IDMap struct {
	StrToID map[string]int
	IDToStr map[int]string
}


func NewIdMap() IDMap {
	return IDMap{
		StrToID: make(map[string]int),
		IDToStr: make(map[int]string),
	}
}

func (idMap *IDMap) GetID(str string) int {
	if id, ok := idMap.StrToID[str]; ok {
		return id
	}
	id := len(idMap.StrToID)
	idMap.StrToID[str] = id
	idMap.IDToStr[id] = str
	return id
}

func (idMap *IDMap) GetStr(id int) string {
	if str, ok := idMap.IDToStr[id]; ok {
		return str
	}
	return ""
}




