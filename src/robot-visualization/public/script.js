document.addEventListener('DOMContentLoaded', () => {
    const socket = io();
    const canvas = document.getElementById('robotMap');
    const ctx = canvas.getContext('2d');
    const robotImg1 = document.getElementById('robot1');
    const startButton = document.getElementById('start');
    const stopButton = document.getElementById('stop');
    const stop2Button = document.getElementById('stop2');
    const moveButton = document.getElementById('move');
    const targetXInput = document.getElementById('targetX');
    const targetYInput = document.getElementById('targetY');
    const longitudinalInput = document.getElementById('longitudinal_velocity');
    const rotationalInput = document.getElementById('rotational_velocity');
    let grid = []; // Define grid at the top level of your script
    const MAP_ORIGIN = 160; // origin point for the map
    const GRID_SIZE  = 50 // mm
    let selectedRobotId = 1; // Default selected robot

    function drawGrid() {
        if (!grid || !grid.length) {
            console.log("Grid is undefined or empty.");
            return; // Early return to prevent TypeError
        }

        const numRows = grid.length;
        const numCols = grid[0].length;
        const cellSize = canvas.width / numCols; // Assuming a square grid and canvas
        console.log('cellSize ', cellSize , numRows, numCols);

        ctx.clearRect(0, 0, canvas.width, canvas.height); // Clear canvas before drawing

        for (let row = 0; row < numRows; row++) {
            for (let col = 0; col < numCols; col++) {
                ctx.fillStyle = grid[row][col] === -1 ? '#D3D3D3' :
                    grid[row][col] === 0 ? '#FFFFFF' : '#FF0000';
                ctx.fillRect(col * cellSize, row * cellSize, cellSize, cellSize);
            }
        }

        // Draw grid lines every 20 cells
        ctx.strokeStyle = '#B0B0B0'; // Color of the grid lines
        ctx.lineWidth = 1; // Width of the grid lines
        for (let i = 0; i <= numCols; i += 20) {
            ctx.beginPath();
            ctx.moveTo(i * cellSize, 0);
            ctx.lineTo(i * cellSize, canvas.height);
            ctx.stroke();
        }
        for (let i = 0; i <= numRows; i += 20) {
            ctx.beginPath();
            ctx.moveTo(0, i * cellSize);
            ctx.lineTo(canvas.width, i * cellSize);
            ctx.stroke();
        }

        // Draw origin icon
        ctx.fillStyle = '#00FF00'; // Green color for the origin icon
        ctx.fillRect(MAP_ORIGIN * cellSize, MAP_ORIGIN * cellSize, cellSize, cellSize);
    }

    function drawRobot(x, y, heading, id) {
        const cellSize = canvas.width / grid[0].length; // Assuming grid is already defined
        x = (x / GRID_SIZE) + MAP_ORIGIN;
        y = MAP_ORIGIN - (y / GRID_SIZE);
        const centerX = (x + 0.5) * cellSize;
        const centerY = (y + 0.5) * cellSize;

        const imageSize = cellSize * 9; // Adjust size of the image as needed
        const robotImg = document.getElementById(`robot${id}`);
        // Update the position of the robot image
        robotImg.style.left = (centerX - imageSize / 2) + 'px';
        robotImg.style.top = (centerY - imageSize / 2) + 'px';

        // Optionally, update the rotation of the image based on the heading
        robotImg.style.transform = 'rotate(' + -heading + 'deg)';
    }

    function updateRobotInfoMode(mode) {
        const modes = ["CUSTOM MODE", "GO TO GOAL MODE", "WALL FOLLOWING MODE"];
        modes.forEach((m, index) => {
            const modeElement = document.getElementById(`mode-${index}`);
            if (m === mode) {
                modeElement.classList.add('active');
            } else {
                modeElement.classList.remove('active');
            }
        });
    }

    function updateRobotInfoState(state) {
        const states = ["ADJUST HEADING", "GO STRAIGHT", "GOAL ACHIEVED"];
        states.forEach((s, index) => {
            const stateElement = document.getElementById(`state-${index}`);
            if (s === state) {
                stateElement.classList.add('active');
            } else {
                stateElement.classList.remove('active');
            }
        });
    }

    function updateTargetIcon(x, y) {
        const cellSize = canvas.width / grid[0].length; // Assuming grid is already defined
        x = (x * 1000 / GRID_SIZE) + MAP_ORIGIN;
        y = MAP_ORIGIN - (y * 1000 / GRID_SIZE);
        const centerX = (x + 0.5) * cellSize;
        const centerY = (y + 0.5) * cellSize;
        const targetIcon = document.querySelector('.targetIcon');
        targetIcon.style.left = centerX + 'px';
        targetIcon.style.top = centerY + 'px';
    }

    function updateSensorState(sensorType, sensor, state) {
        const sensorElement = document.getElementById(`${sensorType}${sensor}`);
        if (!sensorElement) return; // Sensor element not found, return early
        console.log('sensorElement', sensorElement);
        sensorElement.classList.remove('pressed', 'released', 'floor', 'cliff');
        sensorElement.classList.add(state.toLowerCase());
    }

    function updatePozyxInfoPos(x, y, heading) {
        const posElement = document.getElementById(`pozyxPos`);
        if (!posElement) return; // Position element not found, return early
        posElement.innerText = `Pozyx: (${x}, ${y}, ${heading})`;
    }

    function updateRobotInfoPos(x, y, heading) {
        const posElement = document.getElementById(`robotPos`);
        if (!posElement) return; // Position element not found, return early
        // Limit the number of characters in the heading field to 10
        if (heading.length > 10) {
            heading = heading.substring(0, 10);
        }
        posElement.innerText = `Odometry: (${x}, ${y}, ${heading})`;
    }

    socket.on('robot coordinates', (position) => {
        if (Number(position.robot_id) === selectedRobotId) {
            updateRobotInfoPos(position.x, position.y, position.heading);
            updatePozyxInfoPos(position.pozyx_x, position.pozyx_y, position.pozyx_heading);
        }
        drawRobot(position.pozyx_x, position.pozyx_y, position.pozyx_heading, position.robot_id); // Draw the robot at the new position

    });

    socket.on('map update', (data) => {
        console.log('Received map');
        if (Number(data.robot_id) === selectedRobotId) {
            // Assuming the message contains a flattened array of integers
            const data2 = new Int32Array(data.grid);
            // Assuming the 2D array was originally a 320x320 map
            const map = [];
            for (let i = 0; i < 320; i++) {
                map.push(Array.from(data2.slice(i * 320, (i + 1) * 320)));
            }
            grid = map; // Assuming the map data is in a field called 'map'
            drawGrid(); // Now safe to call drawGrid
        }

    });

    socket.on('robot mode', (data) => {
        console.log('Received robot mode:', data);
        if (data.robot_id === selectedRobotId) {
            updateRobotInfoMode(data.mode); // Update the robot mode display
        }
    });

    socket.on('moving state', (data) => {
        if (data.robot_id === selectedRobotId) {
            updateRobotInfoState(data.state); // Update the moving state display
        }
    });

    socket.on('bumper state', (data) => {
        if (data.robot_id === selectedRobotId) {
            updateSensorState('bumper', data.bumper, data.state);
            console.log('Received bumper state:', data);
        }
    });

    socket.on('cliff state', (data) => {
        if (data.robot_id === selectedRobotId) {
            updateSensorState('cliff', data.sensor, data.state);
            console.log('Received cliff state:', data);
        }
    });

    startButton.addEventListener('click', () => {
        const x = parseFloat(targetXInput.value); // Convert input value to float
        const y = parseFloat(targetYInput.value);
        const message = { x, y, robot_id: selectedRobotId }; // Create the message object
        updateTargetIcon(x, y);
        socket.emit('start command', message); // Emit the message object to the server
    });

    stopButton.addEventListener('click', () => {
        const message = { robot_id: selectedRobotId }; // Include selectedRobotId
        socket.emit('stop command', message); // Emit a stop command event without additional data
    });

    stop2Button.addEventListener('click', () => {
        const message = { robot_id: selectedRobotId }; // Include selectedRobotId
        socket.emit('stop command', message);
    });

    moveButton.addEventListener('click', () => {
        const longitudinal_velocity = parseFloat(longitudinalInput.value);
        const rotational_velocity = parseFloat(rotationalInput.value);
        const message = { longitudinal_velocity, rotational_velocity, robot_id: selectedRobotId }; // Include selectedRobotId
        socket.emit('move command', message);
    });

    document.getElementById('robotSelector').addEventListener('change', (event) => {
        selectedRobotId = Number(event.target.value); // Convert to number
        // Clear current robot info
        //clearRobotInfo();
        // Request updated info for the selected robot
        //socket.emit('request robot info', { robotId: selectedRobotId });
    });
});
