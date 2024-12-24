package kv

func EncodeWay(sw []SmallWay) ([]byte, error) {
	bb := Encode(sw)

	bbCompressed := bb

	return bbCompressed, nil
}
func LoadWay(bbCompressed []byte) ([]SmallWay, error) {
	var sw []SmallWay

	sw, err := Decode(bbCompressed)

	return sw, err
}

