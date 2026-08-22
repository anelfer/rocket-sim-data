package physics

import (
	"encoding/json"
	"math"
	"strconv"
)

// -----------------------------------------------------------------------------
// Число с плавающей точкой, допускающее «нет значения».
//
// Апоцентр и перицентр не определены, пока траектория не замкнута, и внутри
// модели обозначаются как NaN. Prometheus такое значение принимает и Grafana
// рисует на его месте разрыв, а вот encoding/json на NaN возвращает ошибку
// «unsupported value», и весь ответ REST-эндпоинта превращается в 500.
//
// Этот тип отдаёт наружу null вместо NaN и бесконечностей.
// -----------------------------------------------------------------------------

// Float — число, которое может быть неопределённым.
type Float float64

// MarshalJSON записывает null для NaN и бесконечностей.
func (f Float) MarshalJSON() ([]byte, error) {
	v := float64(f)
	if math.IsNaN(v) || math.IsInf(v, 0) {
		return []byte("null"), nil
	}
	return []byte(strconv.FormatFloat(v, 'g', -1, 64)), nil
}

// UnmarshalJSON читает null как NaN.
func (f *Float) UnmarshalJSON(data []byte) error {
	if string(data) == "null" {
		*f = Float(math.NaN())
		return nil
	}
	var v float64
	if err := json.Unmarshal(data, &v); err != nil {
		return err
	}
	*f = Float(v)
	return nil
}

// Value возвращает значение как обычное число.
func (f Float) Value() float64 { return float64(f) }

// Defined сообщает, определено ли значение.
func (f Float) Defined() bool {
	v := float64(f)
	return !math.IsNaN(v) && !math.IsInf(v, 0)
}
