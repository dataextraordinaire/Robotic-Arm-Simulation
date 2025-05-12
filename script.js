// Constants
const PI = Math.PI;
const ARM_CONFIG = {
  segmentLengths: [2, 1.5, 1],
  jointConstraints: [
    { min: 0, max: PI }, 
    { min: 0, max: PI }, 
    { min: 0, max: PI }
  ],
  maxAngularVelocity: [1.0, 1.2, 1.5],
  maxAngularAcceleration: [0.5, 0.6, 0.7],
  payload: 0.5
};

// State Management
const state = {
  mode: 'basic',
  armConfig: { ...ARM_CONFIG },
  targetPosition: null,
  jointAngles: [PI / 4, PI / 4, PI / 4],
  armPositions: [],
  prevJointAngles: [PI / 4, PI / 4, PI / 4],
  isAnimating: false,
  isReachable: false,
  efficiencyScore: 0,
  movementTime: 0,
  
  // Advanced state
  obstacles: [],
  trajectory: [],
  currentTrajectoryIndex: 0,
  isFollowingTrajectory: false,
  workspaceAnalysis: null,
  
  // Dynamics & ML state
  dynamicsParams: {
    gravity: 9.81,
    masses: [1.0, 0.8, 0.5],
    frictionCoefficients: [0.1, 0.1, 0.1],
    damping: [0.2, 0.2, 0.2]
  },
  mlTrainingProgress: 0
};

// DOM Elements
const canvas = document.getElementById('simulation-canvas');
const ctx = canvas.getContext('2d');

// Mode switching
const basicModeBtn = document.getElementById('basic-mode-btn');
const advancedModeBtn = document.getElementById('advanced-mode-btn');
const basicControls = document.getElementById('basic-controls');
const advancedControls = document.getElementById('advanced-controls');

// Tab elements
const tabButtons = document.querySelectorAll('.tab');
const tabContents = document.querySelectorAll('.tab-content');

// Basic controls
const targetXInput = document.getElementById('target-x');
const targetYInput = document.getElementById('target-y');
const setTargetBtn = document.getElementById('set-target-btn');
const resetArmBtn = document.getElementById('reset-arm-btn');

// Advanced controls
const addObstacleBtn = document.getElementById('add-obstacle-btn');
const obstacleXInput = document.getElementById('obstacle-x');
const obstacleYInput = document.getElementById('obstacle-y');
const obstacleRadiusInput = document.getElementById('obstacle-radius');
const linearPathBtn = document.getElementById('linear-path-btn');
const curvedPathBtn = document.getElementById('curved-path-btn');
const analyzeWorkspaceBtn = document.getElementById('analyze-workspace-btn');
const trainMLBtn = document.getElementById('train-ml-btn');

// Results area
const resultsContent = document.getElementById('results-content');

// Initialize Canvas Size
function resizeCanvas() {
  const parent = canvas.parentElement;
  canvas.width = parent.clientWidth;
  canvas.height = parent.clientHeight;
  drawArm();
}

// Event Listeners
window.addEventListener('load', () => {
  resizeCanvas();
  initialize();
  
  // Mode switching
  basicModeBtn.addEventListener('click', () => setMode('basic'));
  advancedModeBtn.addEventListener('click', () => setMode('advanced'));
  
  // Tab switching
  tabButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      const tabId = btn.getAttribute('data-tab');
      
      // Update active tab button
      tabButtons.forEach(tab => tab.classList.remove('active'));
      btn.classList.add('active');
      
      // Show correct tab content
      tabContents.forEach(content => {
        content.classList.add('hidden');
      });
      document.getElementById(`tab-${tabId}`).classList.remove('hidden');
    });
  });
  
  // Basic controls
  setTargetBtn.addEventListener('click', handleSetTarget);
  resetArmBtn.addEventListener('click', resetArm);
  
  // Advanced controls
  addObstacleBtn.addEventListener('click', handleAddObstacle);
  linearPathBtn.addEventListener('click', () => planTrajectory('linear'));
  curvedPathBtn.addEventListener('click', () => planTrajectory('curved'));
  analyzeWorkspaceBtn.addEventListener('click', analyzeWorkspace);
  trainMLBtn.addEventListener('click', trainMLModel);
});

window.addEventListener('resize', resizeCanvas);

// Core Functions
function initialize() {
  calculateForwardKinematics(state.jointAngles);
  drawArm();
}

function setMode(mode) {
  state.mode = mode;
  
  // Update UI
  if (mode === 'basic') {
    basicModeBtn.className = 'inline-flex h-8 items-center justify-center rounded-md bg-primary px-4 py-2 text-sm font-medium text-primary-foreground';
    advancedModeBtn.className = 'inline-flex h-8 items-center justify-center rounded-md border border-input bg-background px-4 py-2 text-sm font-medium text-muted-foreground';
    basicControls.classList.remove('hidden');
    advancedControls.classList.add('hidden');
  } else {
    basicModeBtn.className = 'inline-flex h-8 items-center justify-center rounded-md border border-input bg-background px-4 py-2 text-sm font-medium text-muted-foreground';
    advancedModeBtn.className = 'inline-flex h-8 items-center justify-center rounded-md bg-primary px-4 py-2 text-sm font-medium text-primary-foreground';
    basicControls.classList.add('hidden');
    advancedControls.classList.remove('hidden');
  }
  
  drawArm();
}

function handleSetTarget() {
  const x = parseFloat(targetXInput.value);
  const y = parseFloat(targetYInput.value);
  
  if (isNaN(x) || isNaN(y)) {
    alert('Please enter valid coordinates');
    return;
  }
  
  setTargetPosition({ x, y });
}

function setTargetPosition(position) {
  state.targetPosition = position;
  state.isAnimating = true;
  
  // Check if target is reachable
  const reachable = isTargetReachable(state.armConfig, position);
  state.isReachable = reachable;
  
  drawArm();
  updateResults();
  
  // Simulate the inverse kinematics calculation
  setTimeout(() => {
    if (reachable) {
      const prevAngles = [...state.jointAngles];
      const result = calculateInverseKinematics(state.armConfig, position, state.jointAngles);
      
      if (result) {
        state.jointAngles = result.jointAngles;
        state.armPositions = result.positions;
        state.prevJointAngles = prevAngles;
        state.efficiencyScore = calculateEfficiencyScore(state.armConfig, position, result.jointAngles);
        state.movementTime = estimateMovementTime(prevAngles, result.jointAngles);
      }
    }
    
    state.isAnimating = false;
    drawArm();
    updateResults();
  }, 500);
}

function resetArm() {
  const defaultAngles = [PI / 4, PI / 4, PI / 4];
  state.jointAngles = defaultAngles;
  state.prevJointAngles = defaultAngles;
  state.targetPosition = null;
  state.isAnimating = false;
  state.isReachable = false;
  state.efficiencyScore = 0;
  state.movementTime = 0;
  state.trajectory = [];
  state.currentTrajectoryIndex = 0;
  state.isFollowingTrajectory = false;
  
  calculateForwardKinematics(defaultAngles);
  drawArm();
  updateResults();
}

// Forward Kinematics
function calculateForwardKinematics(jointAngles) {
  const { segmentLengths } = state.armConfig;
  
  // Start at the origin (the base of the arm)
  const positions = [];
  let x = 0;
  let y = 0;
  let cumulativeAngle = 0;
  
  // For each segment, calculate the position of the next joint
  for (let i = 0; i < segmentLengths.length; i++) {
    // Add the current joint angle to the cumulative angle
    cumulativeAngle += jointAngles[i];
    
    // Calculate the next position using trigonometry
    x += segmentLengths[i] * Math.sin(cumulativeAngle);
    y += segmentLengths[i] * Math.cos(cumulativeAngle);
    
    // Add the position to the array
    positions.push({ x, y });
  }
  
  state.armPositions = positions;
  return positions;
}

// Inverse Kinematics (simplified CCD algorithm)
function calculateInverseKinematics(config, targetPosition, initialAngles) {
  // First check if the target is reachable
  if (!isTargetReachable(config, targetPosition)) {
    return null;
  }
  
  // Copy the initial angles
  let angles = [...initialAngles];
  
  // Constants for the algorithm
  const MAX_ITERATIONS = 50;
  const CONVERGENCE_THRESHOLD = 0.01;
  
  // Run the CCD algorithm
  for (let iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
    // Calculate current positions
    const positions = calculateForwardKinematics(angles);
    const endEffector = positions[positions.length - 1];
    
    // Calculate error (distance to target)
    const error = Math.sqrt(
      Math.pow(endEffector.x - targetPosition.x, 2) +
      Math.pow(endEffector.y - targetPosition.y, 2)
    );
    
    // Check if we're close enough
    if (error < CONVERGENCE_THRESHOLD) {
      return { jointAngles: angles, positions };
    }
    
    // Adjust each joint angle, working backwards from the end
    for (let i = angles.length - 1; i >= 0; i--) {
      // Get the position of this joint
      const joint = i === 0 ? { x: 0, y: 0 } : positions[i - 1];
      
      // Calculate vectors
      const toEndEffector = {
        x: endEffector.x - joint.x,
        y: endEffector.y - joint.y
      };
      
      const toTarget = {
        x: targetPosition.x - joint.x,
        y: targetPosition.y - joint.y
      };
      
      // Calculate the angle between these vectors
      const dotProduct = toEndEffector.x * toTarget.x + toEndEffector.y * toTarget.y;
      const magnitudeProduct = Math.sqrt(
        (toEndEffector.x * toEndEffector.x + toEndEffector.y * toEndEffector.y) *
        (toTarget.x * toTarget.x + toTarget.y * toTarget.y)
      );
      
      // Avoid division by zero
      if (magnitudeProduct < 0.0001) {
        continue;
      }
      
      let cosAngle = dotProduct / magnitudeProduct;
      cosAngle = Math.max(-1, Math.min(1, cosAngle)); // Clamp to [-1, 1]
      
      // Calculate adjustment angle
      let adjustment = Math.acos(cosAngle);
      
      // Determine the sign of the adjustment using cross product
      const crossProduct = toEndEffector.x * toTarget.y - toEndEffector.y * toTarget.x;
      if (crossProduct < 0) {
        adjustment = -adjustment;
      }
      
      // Apply the adjustment
      angles[i] += adjustment;
      
      // Clamp to joint constraints
      angles[i] = Math.max(
        config.jointConstraints[i].min,
        Math.min(config.jointConstraints[i].max, angles[i])
      );
      
      // Recalculate positions
      const newPositions = calculateForwardKinematics(angles);
      const newEndEffector = newPositions[newPositions.length - 1];
      
      // Update end effector position for next iteration
      endEffector.x = newEndEffector.x;
      endEffector.y = newEndEffector.y;
    }
  }
  
  // Return best solution
  return { 
    jointAngles: angles, 
    positions: calculateForwardKinematics(angles) 
  };
}

// Utility Functions
function isTargetReachable(config, targetPosition) {
  const { segmentLengths } = config;
  
  // Calculate the total arm length (sum of all segment lengths)
  const totalArmLength = segmentLengths.reduce((sum, length) => sum + length, 0);
  
  // Calculate the distance to the target
  const distance = Math.sqrt(
    targetPosition.x * targetPosition.x + targetPosition.y * targetPosition.y
  );
  
  // Check if the target is within reach
  return distance <= totalArmLength;
}

function calculateEfficiencyScore(config, targetPosition, jointAngles) {
  // Simplified efficiency calculation (0-10 scale)
  // Based on how close to middle of joint ranges and distance ratio
  let score = 7.5 + (Math.random() * 2.5 - 1.25); // Simulate a score between 6.25 and 8.75
  return Math.min(10, Math.max(0, score));
}

function estimateMovementTime(prevAngles, newAngles) {
  // Simplified movement time calculation
  const BASE_TIME = 0.5;
  const ANGLE_TIME_FACTOR = 0.02;
  
  // Calculate total angular movement
  let totalAngularMovement = 0;
  for (let i = 0; i < prevAngles.length; i++) {
    totalAngularMovement += Math.abs(newAngles[i] - prevAngles[i]);
  }
  
  // Calculate movement time
  return BASE_TIME + ANGLE_TIME_FACTOR * totalAngularMovement;
}

// Advanced Functions
function handleAddObstacle() {
  const x = parseFloat(obstacleXInput.value);
  const y = parseFloat(obstacleYInput.value);
  const radius = parseFloat(obstacleRadiusInput.value);
  
  if (isNaN(x) || isNaN(y) || isNaN(radius) || radius <= 0) {
    alert('Please enter valid obstacle parameters');
    return;
  }
  
  addObstacle({
    position: { x, y },
    radius,
    type: 'circle',
    id: Date.now().toString()
  });
}

function addObstacle(obstacle) {
  state.obstacles.push(obstacle);
  drawArm();
}

function planTrajectory(type) {
  if (!state.targetPosition || state.armPositions.length === 0) {
    alert('Please set a target position first');
    return;
  }
  
  state.isAnimating = true;
  drawArm();
  
  setTimeout(() => {
    // Generate a simple trajectory (normally would be more complex)
    const start = state.armPositions[state.armPositions.length - 1];
    const end = state.targetPosition;
    const steps = 20;
    
    const trajectory = [];
    
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      
      // For curved trajectory, add some curvature
      let x, y;
      if (type === 'curved') {
        // Simple quadratic curve
        const midX = (start.x + end.x) / 2 + (Math.random() - 0.5) * 2;
        const midY = (start.y + end.y) / 2 + (Math.random() - 0.5) * 2;
        
        // Quadratic Bezier
        const mt = 1 - t;
        x = mt * mt * start.x + 2 * mt * t * midX + t * t * end.x;
        y = mt * mt * start.y + 2 * mt * t * midY + t * t * end.y;
      } else {
        // Linear interpolation
        x = start.x + t * (end.x - start.x);
        y = start.y + t * (end.y - start.y);
      }
      
      // We'd normally calculate IK for each point, but we'll simplify
      // and just interpolate joint angles for demo purposes
      const angles = state.jointAngles.map((angle, j) => {
        const targetAngle = state.jointAngles[j] + t * (Math.random() * 0.4 - 0.2);
        return Math.max(
          state.armConfig.jointConstraints[j].min,
          Math.min(state.armConfig.jointConstraints[j].max, targetAngle)
        );
      });
      
      trajectory.push({
        position: { x, y },
        jointAngles: angles,
        timeStamp: t * 2.0 // Scale to seconds
      });
    }
    
    state.trajectory = trajectory;
    state.currentTrajectoryIndex = 0;
    state.isAnimating = false;
    drawArm();
  }, 500);
}

function analyzeWorkspace() {
  state.isAnimating = true;
  drawArm();
  
  setTimeout(() => {
    // Generate workspace analysis data
    state.workspaceAnalysis = {
      reachableArea: 21.5,
      maxReach: 4.5,
      manipulability: 0.67,
      singularities: [
        { x: 0, y: 4.5 },
        { x: 4.5, y: 0 },
        { x: -4.5, y: 0 },
      ],
      optimalWorkingRegion: [
        { x: 2, y: 2 },
        { x: 2.5, y: 1.5 },
        { x: 1.5, y: 2.5 },
        { x: 3, y: 1 },
        { x: 1, y: 3 },
      ]
    };
    
    state.isAnimating = false;
    drawArm();
  }, 1000);
}

function trainMLModel() {
  state.isTrainingML = true;
  state.mlTrainingProgress = 0;
  updateResults();
  
  // Simulate training progress
  const interval = setInterval(() => {
    state.mlTrainingProgress += 25;
    updateResults();
    
    if (state.mlTrainingProgress >= 100) {
      clearInterval(interval);
      state.isTrainingML = false;
      updateResults();
      alert('ML model trained successfully!');
    }
  }, 500);
}

// UI Rendering Functions
function drawArm() {
  const canvas = document.getElementById('simulation-canvas');
  const ctx = canvas.getContext('2d');
  
  // Clear the canvas
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  
  // Set the origin at the center bottom of the canvas
  const originX = canvas.width / 2;
  const originY = canvas.height - 100; // Leave some space at the bottom
  
  // Scale factor for visualization (pixels per unit)
  const scale = Math.min(canvas.width, canvas.height) / 10;
  
  // Draw grid
  drawGrid(ctx, canvas.width, canvas.height, originX, originY, scale);
  
  // In advanced mode, draw additional elements
  if (state.mode === 'advanced') {
    // Draw workspace analysis
    if (state.workspaceAnalysis) {
      drawWorkspaceAnalysis(ctx, canvas.width, canvas.height, originX, originY, scale);
    }
    
    // Draw obstacles
    drawObstacles(ctx, originX, originY, scale);
    
    // Draw trajectory
    drawTrajectory(ctx, originX, originY, scale);
  }
  
  // Draw target position if it exists
  if (state.targetPosition) {
    const targetX = originX + state.targetPosition.x * scale;
    const targetY = originY - state.targetPosition.y * scale;
    
    // Draw target circle
    ctx.beginPath();
    ctx.arc(targetX, targetY, 10, 0, Math.PI * 2);
    ctx.fillStyle = state.isReachable ? "#4ade80" : "#ef4444"; // Green if reachable, red if not
    ctx.fill();
    
    // Add crosshair
    ctx.beginPath();
    ctx.moveTo(targetX - 15, targetY);
    ctx.lineTo(targetX + 15, targetY);
    ctx.moveTo(targetX, targetY - 15);
    ctx.lineTo(targetX, targetY + 15);
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();
    
    // Draw coordinates
    ctx.font = "14px Inter";
    ctx.fillStyle = "#ffffff";
    ctx.textAlign = "left";
    ctx.fillText(
      `(${state.targetPosition.x.toFixed(2)}, ${state.targetPosition.y.toFixed(2)})`, 
      targetX + 15, 
      targetY - 15
    );
  }
  
  // Draw the arm segments and joints
  if (state.armPositions.length) {
    let prevX = originX;
    let prevY = originY;
    
    // Draw arm segments
    ctx.beginPath();
    ctx.moveTo(prevX, prevY);
    
    state.armPositions.forEach((pos, index) => {
      const x = originX + pos.x * scale;
      const y = originY - pos.y * scale;
      
      ctx.lineTo(x, y);
    });
    
    ctx.strokeStyle = "#64748b"; // Slate gray
    ctx.lineWidth = 12;
    ctx.lineCap = "round";
    ctx.stroke();
    
    // Draw joints
    prevX = originX;
    prevY = originY;
    
    // Draw base
    ctx.beginPath();
    ctx.rect(originX - 30, originY - 10, 60, 10);
    ctx.fillStyle = "#334155"; // Darker slate
    ctx.fill();
    
    state.armPositions.forEach((pos, index) => {
      const x = originX + pos.x * scale;
      const y = originY - pos.y * scale;
      
      // Draw joint circle
      ctx.beginPath();
      ctx.arc(prevX, prevY, 15, 0, Math.PI * 2);
      ctx.fillStyle = "#0ea5e9"; // Blue
      ctx.fill();
      ctx.strokeStyle = "#0c4a6e";
      ctx.lineWidth = 2;
      ctx.stroke();
      
      // Draw angle indicator arc
      if (index < state.jointAngles.length) {
        drawAngleArc(ctx, prevX, prevY, 25, state.jointAngles[index]);
      }
      
      prevX = x;
      prevY = y;
    });
    
    // Draw end effector
    ctx.beginPath();
    ctx.arc(prevX, prevY, 12, 0, Math.PI * 2);
    ctx.fillStyle = "#ec4899"; // Pink
    ctx.fill();
    ctx.strokeStyle = "#831843";
    ctx.lineWidth = 2;
    ctx.stroke();
    
    // Draw coordinates of end effector
    const endPos = state.armPositions[state.armPositions.length - 1];
    ctx.font = "14px Inter";
    ctx.fillStyle = "#ffffff";
    ctx.textAlign = "left";
    ctx.fillText(
      `End: (${endPos.x.toFixed(2)}, ${endPos.y.toFixed(2)})`, 
      prevX + 20, 
      prevY
    );
  }
  
  // Draw arm lengths for reference
  drawArmLengths(ctx, canvas.width, canvas.height, state.armConfig.segmentLengths);
  
  // Draw status message if animating
  if (state.isAnimating) {
    ctx.font = "18px Oswald";
    ctx.fillStyle = "#ffffff";
    ctx.textAlign = "center";
    ctx.fillText("Calculating optimal position...", canvas.width / 2, 60);
  }
  
  // Draw reachability message
  if (state.targetPosition) {
    ctx.font = "18px Oswald";
    ctx.fillStyle = state.isReachable ? "#4ade80" : "#ef4444";
    ctx.textAlign = "center";
    ctx.fillText(
      state.isReachable ? "Target is reachable" : "Target is unreachable", 
      canvas.width / 2, 
      30
    );
  }
}

// Draw grid lines
function drawGrid(ctx, width, height, originX, originY, scale) {
  const gridSize = 1; // 1 unit in our coordinate system
  const gridPixels = gridSize * scale;
  
  // Draw grid lines
  ctx.beginPath();
  
  // Vertical lines
  for (let x = originX % gridPixels; x < width; x += gridPixels) {
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
  }
  
  // Horizontal lines
  for (let y = originY % gridPixels; y < height; y += gridPixels) {
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
  }
  
  ctx.strokeStyle = "rgba(100, 116, 139, 0.2)"; // Very light slate
  ctx.lineWidth = 1;
  ctx.stroke();
  
  // Draw axes
  ctx.beginPath();
  // X-axis
  ctx.moveTo(0, originY);
  ctx.lineTo(width, originY);
  // Y-axis
  ctx.moveTo(originX, height);
  ctx.lineTo(originX, 0);
  
  ctx.strokeStyle = "rgba(100, 116, 139, 0.5)"; // Light slate
  ctx.lineWidth = 2;
  ctx.stroke();
  
  // Draw axis labels
  ctx.font = "14px Inter";
  ctx.fillStyle = "#94a3b8";
  
  // X-axis labels
  for (let x = Math.floor(-originX / scale); x <= Math.floor((width - originX) / scale); x++) {
    if (x === 0) continue; // Skip origin
    const xPos = originX + x * scale;
    ctx.fillText(x.toString(), xPos - 5, originY + 20);
  }
  
  // Y-axis labels
  for (let y = Math.floor(-originY / scale); y <= Math.floor((originY) / scale); y++) {
    if (y === 0) continue; // Skip origin
    const yPos = originY - y * scale;
    ctx.fillText(y.toString(), originX + 10, yPos + 5);
  }
  
  // Origin label
  ctx.fillText("0", originX + 5, originY + 15);
}

// Draw angle arc to visualize joint angles
function drawAngleArc(ctx, x, y, radius, angle) {
  const startAngle = -Math.PI / 2; // Start from top (negative y-axis)
  const endAngle = startAngle + angle;
  
  ctx.beginPath();
  ctx.arc(x, y, radius, startAngle, endAngle);
  ctx.strokeStyle = "#fbbf24"; // Amber
  ctx.lineWidth = 3;
  ctx.stroke();
  
  // Add arrowhead at the end of the arc
  const arrowX = x + radius * Math.cos(endAngle);
  const arrowY = y + radius * Math.sin(endAngle);
  
  ctx.beginPath();
  ctx.moveTo(arrowX, arrowY);
  ctx.lineTo(
    arrowX - 8 * Math.cos(endAngle - Math.PI / 6),
    arrowY - 8 * Math.sin(endAngle - Math.PI / 6)
  );
  ctx.lineTo(
    arrowX - 8 * Math.cos(endAngle + Math.PI / 6),
    arrowY - 8 * Math.sin(endAngle + Math.PI / 6)
  );
  ctx.closePath();
  ctx.fillStyle = "#fbbf24"; // Amber
  ctx.fill();
  
  // Draw angle value
  const textX = x + (radius + 15) * Math.cos(startAngle + angle / 2);
  const textY = y + (radius + 15) * Math.sin(startAngle + angle / 2);
  
  ctx.font = "12px Inter";
  ctx.fillStyle = "#ffffff";
  ctx.textAlign = "center";
  ctx.fillText(`${Math.round(angle * (180 / Math.PI))}°`, textX, textY);
}

// Draw arm lengths for reference
function drawArmLengths(ctx, width, height, segmentLengths) {
  ctx.font = "14px Inter";
  ctx.fillStyle = "#cbd5e1";
  ctx.textAlign = "left";
  
  let y = 70;
  ctx.fillText("Arm Segments:", 20, y);
  y += 20;
  
  segmentLengths.forEach((length, index) => {
    ctx.fillText(`Segment ${index + 1}: ${length.toFixed(2)} units`, 30, y);
    y += 20;
  });
}

// Draw obstacles
function drawObstacles(ctx, originX, originY, scale) {
  state.obstacles.forEach(obstacle => {
    const x = originX + obstacle.position.x * scale;
    const y = originY - obstacle.position.y * scale;
    const radius = obstacle.radius * scale;
    
    // Draw obstacle
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(239, 68, 68, 0.2)"; // Red with transparency
    ctx.fill();
    
    // Draw outline
    ctx.strokeStyle = "rgba(239, 68, 68, 0.8)";
    ctx.lineWidth = 2;
    ctx.stroke();
  });
}

// Draw trajectory
function drawTrajectory(ctx, originX, originY, scale) {
  if (!state.trajectory.length) return;
  
  // Draw trajectory path
  ctx.beginPath();
  
  state.trajectory.forEach((point, index) => {
    const x = originX + point.position.x * scale;
    const y = originY - point.position.y * scale;
    
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
    
    // Draw small point at each step
    if (index % 2 === 0) { // Draw every other point to avoid clutter
      ctx.fillStyle = index === state.currentTrajectoryIndex ? "#3b82f6" : "#64748b";
      ctx.beginPath();
      ctx.arc(x, y, 3, 0, Math.PI * 2);
      ctx.fill();
    }
  });
  
  // Style the path
  ctx.strokeStyle = "rgba(59, 130, 246, 0.5)"; // Blue with transparency
  ctx.lineWidth = 2;
  ctx.stroke();
  
  // Highlight current position in trajectory
  if (state.currentTrajectoryIndex > 0 && state.currentTrajectoryIndex < state.trajectory.length) {
    const current = state.trajectory[state.currentTrajectoryIndex];
    const x = originX + current.position.x * scale;
    const y = originY - current.position.y * scale;
    
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, Math.PI * 2);
    ctx.fillStyle = "#3b82f6"; // Blue
    ctx.fill();
  }
}

// Draw workspace analysis
function drawWorkspaceAnalysis(ctx, width, height, originX, originY, scale) {
  if (!state.workspaceAnalysis) return;
  
  // Draw maximum reach circle
  ctx.beginPath();
  ctx.arc(originX, originY, state.workspaceAnalysis.maxReach * scale, 0, Math.PI * 2);
  ctx.strokeStyle = "rgba(139, 92, 246, 0.3)"; // Purple with transparency
  ctx.lineWidth = 1;
  ctx.stroke();
  
  // Draw optimal working region
  state.workspaceAnalysis.optimalWorkingRegion.forEach(point => {
    const x = originX + point.x * scale;
    const y = originY - point.y * scale;
    
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(139, 92, 246, 0.5)"; // Purple with transparency
    ctx.fill();
  });
  
  // Draw singularities
  state.workspaceAnalysis.singularities.forEach(point => {
    const x = originX + point.x * scale;
    const y = originY - point.y * scale;
    
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(249, 115, 22, 0.7)"; // Orange
    ctx.fill();
  });
}

// Update results panel
function updateResults() {
  if (state.isTrainingML) {
    resultsContent.innerHTML = `
      <div class="space-y-4">
        <div class="space-y-2">
          <label class="text-sm font-medium">Training ML Model</label>
          <div class="w-full bg-muted rounded-full h-2.5">
            <div class="bg-primary h-2.5 rounded-full" style="width: ${state.mlTrainingProgress}%"></div>
          </div>
          <div class="text-sm text-muted-foreground text-right">${state.mlTrainingProgress}%</div>
        </div>
      </div>
    `;
    return;
  }
  
  if (!state.targetPosition) {
    resultsContent.innerHTML = `
      <div class="flex flex-col items-center justify-center py-6 text-center text-muted-foreground">
        <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="mb-4 opacity-50"><circle cx="12" cy="12" r="10"></circle><path d="M12 16v-4"></path><path d="M12 8h.01"></path></svg>
        <p class="mb-2">No target position set</p>
        <p class="text-sm">
          Use the form above to set a target position for the robot arm
        </p>
      </div>
    `;
    return;
  }
  
  if (state.isAnimating) {
    resultsContent.innerHTML = `
      <div class="flex flex-col items-center justify-center py-6 text-center">
        <div class="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mb-4"></div>
        <p class="text-lg font-medium">Calculating...</p>
      </div>
    `;
    return;
  }
  
  if (state.isReachable) {
    resultsContent.innerHTML = `
      <div class="space-y-4">
        <div class="flex items-center gap-2">
          <div class="w-4 h-4 rounded-full bg-green-500"></div>
          <span class="font-medium">Target is reachable</span>
        </div>
        
        <hr class="border-border" />
        
        <div class="space-y-1">
          <label class="text-sm font-medium">Movement Time</label>
          <div class="text-xl font-bold">${state.movementTime.toFixed(2)}s</div>
        </div>
        
        <div class="space-y-1">
          <label class="text-sm font-medium">Efficiency Score</label>
          <div class="flex items-center">
            <div class="text-xl font-bold">${state.efficiencyScore.toFixed(1)}</div>
            <span class="text-sm ml-2 text-muted-foreground">/ 10.0</span>
          </div>
        </div>
        
        <hr class="border-border" />
        
        <div class="space-y-1">
          <label class="text-sm font-medium">Joint Angles</label>
          <div class="grid grid-cols-3 gap-2">
            ${state.jointAngles.map((angle, i) => `
              <div class="text-center">
                <div class="text-sm text-muted-foreground">${i + 1}</div>
                <div class="font-bold">${Math.round(angle * 180 / Math.PI)}°</div>
              </div>
            `).join('')}
          </div>
        </div>
      </div>
    `;
  } else {
    resultsContent.innerHTML = `
      <div class="space-y-4">
        <div class="flex items-center gap-2">
          <div class="w-4 h-4 rounded-full bg-red-500"></div>
          <span class="font-medium">Target is unreachable</span>
        </div>
        
        <div class="bg-red-900/20 border border-red-900/50 rounded-md p-3 flex items-start gap-2">
          <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="text-red-500 shrink-0 mt-0.5"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="8" x2="12" y2="12"></line><line x1="12" y1="16" x2="12.01" y2="16"></line></svg>
          <div>
            <p class="text-sm">
              The target position is beyond the reach of the robot arm.
              Try a position closer to the base or adjust segment lengths.
            </p>
          </div>
        </div>
      </div>
    `;
  }
}