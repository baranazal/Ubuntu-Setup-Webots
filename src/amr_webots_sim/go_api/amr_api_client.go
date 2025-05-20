package main

import (
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"net/http"
	"os"
	"time"
)

const (
	baseURL = "http://localhost:5000/api"
)

type AMRClient struct {
	httpClient *http.Client
}

func NewAMRClient() *AMRClient {
	return &AMRClient{
		httpClient: &http.Client{
			Timeout: 10 * time.Second,
		},
	}
}

func (c *AMRClient) makeRequest(method, endpoint string, requestBody interface{}, responseObj interface{}) error {
	url := fmt.Sprintf("%s%s", baseURL, endpoint)

	var reqBodyReader io.Reader
	if requestBody != nil {
		jsonBody, err := json.Marshal(requestBody)
		if err != nil {
			return fmt.Errorf("error marshaling request body: %w", err)
		}
		reqBodyReader = bytes.NewBuffer(jsonBody)
	}

	req, err := http.NewRequest(method, url, reqBodyReader)
	if err != nil {
		return fmt.Errorf("error creating request: %w", err)
	}

	if requestBody != nil {
		req.Header.Set("Content-Type", "application/json")
	}

	resp, err := c.httpClient.Do(req)
	if err != nil {
		return fmt.Errorf("error making request: %w", err)
	}
	defer resp.Body.Close()

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return fmt.Errorf("error reading response body: %w", err)
	}

	if resp.StatusCode < 200 || resp.StatusCode >= 300 {
		return fmt.Errorf("API returned non-success status code %d: %s", resp.StatusCode, string(body))
	}

	if responseObj != nil {
		if err := json.Unmarshal(body, responseObj); err != nil {
			return fmt.Errorf("error unmarshaling response: %w", err)
		}
	}

	return nil
}

func (c *AMRClient) MoveAMR(robotID string, x, y, z float64) error {
	reqBody := map[string]interface{}{
		"robot_id": robotID,
		"x": x,
		"y": y,
		"z": z,
	}
	return c.makeRequest("POST", "/movement/AMR", reqBody, nil)
}

type ArmSequenceStep struct {
	Action        string    `json:"action"`
	Movement      string    `json:"movement,omitempty"`
	JointPositions []float64 `json:"joint_positions,omitempty"`
	Position      float64   `json:"position,omitempty"`
	Delay         float64   `json:"delay"`
}

func (c *AMRClient) MoveRoboticArm(sequence []ArmSequenceStep) error {
	reqBody := map[string]interface{}{
		"sequence": sequence,
	}
	return c.makeRequest("POST", "/movement/robotic_arm", reqBody, nil)
}

func (c *AMRClient) ControlConveyorBelt(speed float64, direction string, running bool) error {
	reqBody := map[string]interface{}{
		"speed":     speed,
		"direction": direction,
		"running":   running,
	}
	return c.makeRequest("POST", "/movement/conveyor_belt", reqBody, nil)
}

func main() {
	client := NewAMRClient()

	moveAMRCmd := flag.NewFlagSet("move-amr", flag.ExitOnError)
	moveRobotID := moveAMRCmd.String("robot", "", "Robot ID to move")
	x := moveAMRCmd.Float64("x", 0.0, "Target X coordinate")
	y := moveAMRCmd.Float64("y", 0.0, "Target Y coordinate")
	z := moveAMRCmd.Float64("z", 0.0, "Target Z coordinate")

	moveArmCmd := flag.NewFlagSet("move-arm", flag.ExitOnError)
	armSequenceFile := moveArmCmd.String("sequence", "", "JSON file with arm sequence")
	
	beltCmd := flag.NewFlagSet("belt", flag.ExitOnError)
	beltSpeed := beltCmd.Float64("speed", 0.25, "Belt speed")
	beltDirection := beltCmd.String("direction", "forward", "Belt direction (forward or reverse)")
	beltRunning := beltCmd.Bool("running", true, "Belt running state (true/false)")

	if len(os.Args) < 2 {
		fmt.Println("Expected subcommand: move-amr, move-arm, or belt")
		os.Exit(1)
	}

	switch os.Args[1] {
	case "move-amr":
		moveAMRCmd.Parse(os.Args[2:])
		if *moveRobotID == "" {
			fmt.Println("Robot ID is required for moving AMR")
			os.Exit(1)
		}
		err := client.MoveAMR(*moveRobotID, *x, *y, *z)
		if err != nil {
			fmt.Printf("Error moving AMR: %v\n", err)
			os.Exit(1)
		}
		fmt.Printf("Moving robot %s to coordinates: x=%.2f, y=%.2f, z=%.2f\n",
			*moveRobotID, *x, *y, *z)

	case "move-arm":
		moveArmCmd.Parse(os.Args[2:])
		if *armSequenceFile == "" {
			fmt.Println("Sequence file path is required for arm movement")
			os.Exit(1)
		}
		
		sequenceData, err := os.ReadFile(*armSequenceFile)
		if err != nil {
			fmt.Printf("Error reading sequence file: %v\n", err)
			os.Exit(1)
		}
		
		var sequence []ArmSequenceStep
		err = json.Unmarshal(sequenceData, &sequence)
		if err != nil {
			fmt.Printf("Error parsing sequence JSON: %v\n", err)
			os.Exit(1)
		}
		
		err = client.MoveRoboticArm(sequence)
		if err != nil {
			fmt.Printf("Error moving robotic arm: %v\n", err)
			os.Exit(1)
		}
		fmt.Printf("Executing arm sequence with %d steps\n", len(sequence))

	case "belt":
		beltCmd.Parse(os.Args[2:])
		err := client.ControlConveyorBelt(*beltSpeed, *beltDirection, *beltRunning)
		if err != nil {
			fmt.Printf("Error controlling conveyor belt: %v\n", err)
			os.Exit(1)
		}
		if *beltRunning {
			fmt.Printf("Belt moving in %s direction at speed %.2f\n", *beltDirection, *beltSpeed)
		} else {
			fmt.Println("Belt stopped")
		}

	default:
		fmt.Println("Expected subcommand: move-amr, move-arm, or belt")
		os.Exit(1)
	}
} 