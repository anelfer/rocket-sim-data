// Package physics содержит базовые физические константы, векторную алгебру
// и модели среды (атмосфера, гравитация), общие для всего симулятора.
package physics

import "math"

// Vec3 — трёхмерный вектор. Используется для позиций, скоростей и сил
// в инерциальной геоцентрической системе координат (ECI).
type Vec3 struct {
	X, Y, Z float64
}

func (v Vec3) Add(o Vec3) Vec3 { return Vec3{v.X + o.X, v.Y + o.Y, v.Z + o.Z} }
func (v Vec3) Sub(o Vec3) Vec3 { return Vec3{v.X - o.X, v.Y - o.Y, v.Z - o.Z} }

func (v Vec3) Scale(k float64) Vec3 { return Vec3{v.X * k, v.Y * k, v.Z * k} }

func (v Vec3) Dot(o Vec3) float64 { return v.X*o.X + v.Y*o.Y + v.Z*o.Z }

func (v Vec3) Cross(o Vec3) Vec3 {
	return Vec3{
		X: v.Y*o.Z - v.Z*o.Y,
		Y: v.Z*o.X - v.X*o.Z,
		Z: v.X*o.Y - v.Y*o.X,
	}
}

func (v Vec3) Norm() float64 { return math.Sqrt(v.Dot(v)) }

// Unit возвращает единичный вектор того же направления.
// Для нулевого вектора возвращает нулевой вектор (без NaN).
func (v Vec3) Unit() Vec3 {
	n := v.Norm()
	if n < 1e-12 {
		return Vec3{}
	}
	return v.Scale(1 / n)
}

// IsFinite сообщает, что все компоненты конечны. Используется как
// страховка от того, что NaN/Inf незаметно расползётся по состоянию.
func (v Vec3) IsFinite() bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}
