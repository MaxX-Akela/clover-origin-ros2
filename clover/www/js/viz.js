var ros = new ROSLIB.Ros({
	url : 'ws://' + location.hostname + ':9090'
});

var titleEl = document.querySelector('title');

ros.on('error', function(error) {
	titleEl.innerText = 'Disconnected';
	err = error;
	alert('Connection error: please enable \'rosbridge\' in clover.launch.py!');
});

ros.on('connection', function() {
	console.log('connected');
	titleEl.innerText = 'Connected';
});

ros.on('close', function() {
	console.log('disconnected');
	titleEl.innerText = 'Disconnected';
});

var viewer, tfClient;

// Replacement for ROSLIB.TFClient: the vendored roslib.js can talk only to the ROS 1 tf2_web_republisher
// (action tf2_web_republisher/TFSubscriptionAction, or the republish_tfs service). The ROS 2 package has
// only the action tf2_web_republisher_interfaces/action/TFSubscription (no service), and the old roslib
// ignores action_feedback messages, so the rosbridge protocol (send_action_goal) is used directly.
// Implements the part of the TFClient interface used by ROS3D: subscribe, unsubscribe, fixedFrame.
function TFActionClient(options) {
	this.ros = options.ros;
	this.fixedFrame = options.fixedFrame || 'base_link';
	this.angularThres = options.angularThres || 2.0;
	this.transThres = options.transThres || 0.01;
	this.rate = options.rate || 10.0;
	this.updateDelay = options.updateDelay || 50;
	this.serverName = options.serverName || '/tf2_web_republisher';
	this.actionType = 'tf2_web_republisher_interfaces/action/TFSubscription';

	this.frameInfos = {};
	this.updateRequested = false;
	this.goalId = null;
	this.goalCounter = 0;
	this.socket = null;

	var that = this;
	this.onMessage = function(event) {
		if (typeof event.data != 'string') return;
		var message = JSON.parse(event.data);
		if (message.op == 'action_feedback' && message.id == that.goalId) {
			that.processTFArray(message.values);
		} else if (message.op == 'action_result' && message.id == that.goalId && !message.result) {
			console.error('tf2_web_republisher: ' + message.values);
		}
	};
	this.attach = function() {
		if (!that.ros.socket || that.ros.socket === that.socket) return;
		that.socket = that.ros.socket;
		that.socket.addEventListener('message', that.onMessage);
	};
	this.ros.on('connection', function() {
		that.attach();
		that.goalId = null; // the goal is lost with the old connection
		if (Object.keys(that.frameInfos).length) that.updateGoal();
	});
	this.attach();
}

TFActionClient.prototype.processTFArray = function(tf) {
	tf.transforms.forEach(function(transform) {
		var info = this.frameInfos[transform.child_frame_id.replace(/^\//, '')];
		if (info) {
			info.transform = new ROSLIB.Transform({
				translation: transform.transform.translation,
				rotation: transform.transform.rotation
			});
			info.cbs.forEach(function(cb) { cb(info.transform); });
		}
	}, this);
};

TFActionClient.prototype.updateGoal = function() {
	this.updateRequested = false;
	if (this.goalId) {
		this.ros.callOnConnection({ op: 'cancel_action_goal', id: this.goalId, action: this.serverName });
	}
	this.goalId = 'tf_goal:' + (++this.goalCounter) + ':' + Date.now();
	this.ros.callOnConnection({
		op: 'send_action_goal',
		id: this.goalId,
		action: this.serverName,
		action_type: this.actionType,
		feedback: true,
		args: {
			source_frames: Object.keys(this.frameInfos),
			target_frame: this.fixedFrame,
			angular_thres: this.angularThres,
			trans_thres: this.transThres,
			rate: this.rate
		}
	});
};

TFActionClient.prototype.requestUpdate = function() {
	if (!this.updateRequested) {
		this.updateRequested = true;
		setTimeout(this.updateGoal.bind(this), this.updateDelay);
	}
};

TFActionClient.prototype.subscribe = function(frameID, callback) {
	frameID = frameID.replace(/^\//, '');
	if (!this.frameInfos[frameID]) {
		this.frameInfos[frameID] = { cbs: [] };
		this.requestUpdate();
	} else if (this.frameInfos[frameID].transform) {
		callback(this.frameInfos[frameID].transform);
	}
	this.frameInfos[frameID].cbs.push(callback);
};

TFActionClient.prototype.unsubscribe = function(frameID, callback) {
	frameID = frameID.replace(/^\//, '');
	var info = this.frameInfos[frameID];
	var cbs = info && info.cbs || [];
	for (var i = cbs.length; i--;) {
		if (cbs[i] === callback) cbs.splice(i, 1);
	}
	if (!callback || cbs.length === 0) {
		delete this.frameInfos[frameID];
	}
};

function setScene(fixedFrame) {
	viewer = new ROS3D.Viewer({
		divID: 'viz',
		width: 1000,
		height: 600,
		antialias: true
	});

	tfClient = new TFActionClient({
		ros: ros,
		angularThres: 0.01,
		transThres: 0.01,
		rate: 10.0,
		fixedFrame : fixedFrame
	});

	var map = new ROS3D.Grid({
		ros: ros,
		tfClient: tfClient,
		rootObject: viewer.scene
	});

	viewer.scene.add(map);
}

function addAxes() {
	var axes = new ROS3D.Axes({
		ros: ros,
		tfClient: tfClient,
		rootObject: viewer.scene
	});
	viewer.scene.add(axes);
}

function addVehicle() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/vehicle_marker',
		rootObject: viewer.scene
	});
}


function addCamera() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/main_camera/camera_markers',
		rootObject: viewer.scene
	});
}

function addAruco() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/aruco_detect/visualization',
		rootObject: viewer.scene
	});
}

function addArucoMap() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/aruco_map/visualization',
		rootObject: viewer.scene
	});
}
