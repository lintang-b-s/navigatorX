package datastructure

import (
	"bytes"
	"fmt"
	"io"

	"github.com/klauspost/compress/zstd"
)

func CompressData(inData []byte, bbufOut *bytes.Buffer) error {

	inputBuf := bytes.NewBuffer(inData)
	encoder, err := zstd.NewWriter(bbufOut, zstd.WithEncoderLevel(zstd.SpeedBestCompression))
	if err != nil {
		return fmt.Errorf("failed to create zstd encoder: %w", err)
	}

	_, err = io.Copy(encoder, inputBuf)
	if err != nil {
		encoder.Close()
		return err
	}
	return encoder.Close()
}

func DecompressData(inData []byte, out io.Writer) error {
	in := bytes.NewBuffer(inData)
	d, err := zstd.NewReader(in)
	if err != nil {
		return err
	}
	defer d.Close()

	// Copy content...
	_, err = io.Copy(out, d)
	return err
}
