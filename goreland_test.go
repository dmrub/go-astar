package astar

import (
	"testing"
)

func AddTruck(x int, y int, label string) *Truck {
	t1 := new(Truck)
	t1.X = x
	t1.Y = y
	t1.label = label
	return t1
}

func AddTube(t1, t2 *Truck, cost float64) *Tube {
	tube1 := new(Tube)
	tube1.Cost = cost
	tube1.from = t1
	tube1.to = t2

	t1.out_to = append(t1.out_to, *tube1)

	return tube1
}

// Consider a world with Nodes (Trucks) and Edges (Tubes), Edges each having a Cost
// 
//    E
//   /|
//  / |
// S--M
// 
// S=Start at (0,0)
// E=End at (1,1)
// M=Middle at (0,1)
// 
// S-M and M-E are clean clear tubes. Cost: 1
// 
// S-E is either:
// 
// 1) TestGraphPath_ShortDiagonal : diagonal is a nice clean clear Tube , Cost: 1.9
//    Solver should traverse the bridge.
//    Expect solution: Start, End  Total Cost: 1.9
// 
// 1) TestGraphPath_LongDiagonal : diagonal is a Tube plugged full of
//    "enormous amounts of material"!, Cost: 10000.
//    Solver should avoid the plugged tube.
//    Expect solution Start,Middle,End  Total Cost: 2.0

func createGorelandGraphPath_Diagonal(t *testing.T, diagonal_cost float64, expectedDist float64) {

	world := new(Goreland)

	tr_start := AddTruck(0, 0, "Start")
	tr_mid := AddTruck(0, 1, "Middle")
	tr_end := AddTruck(1, 1, "End")

	AddTube(tr_start, tr_end, diagonal_cost)
	AddTube(tr_start, tr_mid, 1)
	AddTube(tr_mid, tr_end, 1)

	t.Logf("Goreland.  Diagonal Cost: %v\n\n", diagonal_cost)

	pf := NewPathfinder()
	p, dist, found := pf.Search(tr_start, tr_end)

	if !found {
		t.Log("Could not find a path")
	} else {
		t.Logf("Resulting path\n%s", world.RenderPath(p))
	}
	if !found && expectedDist >= 0 {
		t.Fatal("Could not find a path")
	}
	if found && dist != expectedDist {
		t.Fatalf("Expected dist to be %v but got %v", expectedDist, dist)
	}
}

func TestGraphPaths_ShortDiagonal(t *testing.T) {
	createGorelandGraphPath_Diagonal(t, 1.9, 1.9)
}
func TestGraphPaths_LongDiagonal(t *testing.T) {
	createGorelandGraphPath_Diagonal(t, 10000, 2.0)
}
