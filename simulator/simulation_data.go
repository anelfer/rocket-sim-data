package simulator

type SimulationData struct {
	DryMass            float64 `json:"dryMass"`
	TotalMass          float64 `json:"totalMass"`
	FuelMass           float64 `json:"fuelMass"`
	FuelBurnRate       float64 `json:"fuelBurnRate"`
	VerticalVelocity   float64 `json:"Vvelocity"`
	HorizontalVelocity float64 `json:"Hvelocity"`
	Altitude           float64 `json:"altitude"`
	Gravity            float64 `json:"gravity"`
	Area               float64 `json:"area"`
	DragCoefficient    float64 `json:"dragCoefficient"`
	AirDensitySeaLevel float64 `json:"airDensitySeaLevel"`
	Lat                float64 `json:"lat"`
	Lon                float64 `json:"lon"`
}

// GetData возвращает данные симуляции, включая вычисленные параметры.
func (s *Simulation) GetData() SimulationData {
	if CurrentSimulation == nil {
		// Возвращаем объект с нулевыми или значениями по умолчанию
		return SimulationData{
			DryMass:            0,
			TotalMass:          0,
			FuelMass:           0,
			FuelBurnRate:       0,
			VerticalVelocity:   0,
			HorizontalVelocity: 0,
			Altitude:           0,
			Gravity:            0,
			Area:               0,
			DragCoefficient:    0,
			AirDensitySeaLevel: 0,
			Lat:                0,
			Lon:                0,
		}
	}
	//lat, lon := metrics.GetLatLon(s.Altitude, s.Velocity)
	return SimulationData{
		DryMass:            s.DryMass,
		TotalMass:          s.TotalMass,
		FuelMass:           s.FuelMass,
		VerticalVelocity:   s.VerticalVelocity,
		HorizontalVelocity: s.HorizontalVelocity,
		Altitude:           s.Altitude,
		Gravity:            s.Gravity,
		Area:               s.Area,
		DragCoefficient:    s.DragCoefficient,
		AirDensitySeaLevel: s.AirDensitySeaLevel,
		Lat:                0,
		Lon:                0,
	}
}
