var fisheye;
var video_active = false;

var fetch_fisheye_params = function() {
	if (!fisheye.gl)
		return;

	robairros.get_fisheye_params(function(res) {
		$('#fix-distortion')[0].checked = res.enable;
		$('#distortion-value')[0].value = res.value;

		if ($('#fix-distortion')[0].checked && video_active)
			start_correction();
	});
};

var init_video_funcs = function() {
	var undistorted_video = $('#undistorted-video')[0];
	fisheye = new Fisheye(undistorted_video);

	if (fisheye.gl) {
		$('#video-panel')[0].style.display = '';
		enable_video_controls();
		$('#fix-distortion')[0].addEventListener('change', function() {
			if ($('#fix-distortion')[0].checked && video_active)
				start_correction();
		}, false);

		$('#fix-distortion')[0].addEventListener(
			'change', save_correction_pref, false);

		$('#distortion-value')[0].addEventListener(
			'change', save_correction_val, false);
	}
}

var enable_video_controls = function() {
	$('#fix-distortion')[0].disabled = false;
};

var start_correction = function() {
	$('#undistorted-video')[0].style.display = '';
	$('#remoteVideos')[0].style.display = 'none';
	$('#distortion-value')[0].disabled = false;
	update_video();
};

var stop_correction = function() {
	$('#undistorted-video')[0].style.display = 'none';
	$('#remoteVideos')[0].style.display = '';
	$('#fix-distortion')[0].checked = false;
	$('#distortion-value')[0].disabled = true;
};

var update_video = function() {
	if (!video_active || !$('#fix-distortion')[0].checked) {
		stop_correction();
		return;
	}

	var video = $('#remoteVideos')[0].firstChild;

	fisheye.setDistortion(parseFloat($('#distortion-value')[0].value));
	fisheye.draw(video);

	requestAnimationFrame(update_video);
};

webrtc.on('videoAdded', function() {
	var video = $('#remoteVideos')[0].firstChild;
	if (!video)
		return;

	video_active = true;

	if ($('#fix-distortion')[0].checked)
		setTimeout(start_correction, 500);
});

webrtc.on('videoRemoved', function() {
	video_active = false;
});


var save_correction_pref = function() {
	var val = $('#fix-distortion')[0].checked;
	robairros.enable_fisheye_correction(val);
};

var save_correction_val = function() {
	var val = parseFloat($('#distortion-value')[0].value);
	robairros.set_fisheye_correction(val);
};

var VideoOverlay = {
	_init: function() {
		this._ctx = document.getElementById('overlay-canvas').getContext('2d');
		this._w = document.getElementById('overlay-canvas').width;
		this._h = document.getElementById('overlay-canvas').height;

		this._head_angle = 0;

		this._update();
	},

	set_head: function(angle) {
		this._head_angle = angle;
		this._update();
	},

	_update: function() {
		this._ctx.clearRect(0, 0, this._w, this._h);

		if (this._head_angle <= -5 || this._head_angle >= 5)
			this._draw_heading();
	},


	_HDG_BASE_MULT: 1 / 110,
	_HDG_HORIZON_POINT: {x: 0.5, y: 2},
	_HDG_BASE_HALF_WIDTH: 0.01,
	_HDG_TOP_HALF_WIDTH: 0.003,
	_HDG_LENGTH: 0.20,

	_draw_heading: function() {
		var base_middle_x = -1 * this._head_angle * this._HDG_BASE_MULT + 0.5;
		var base_left_x = base_middle_x - this._HDG_BASE_HALF_WIDTH;
		var base_right_x = base_middle_x + this._HDG_BASE_HALF_WIDTH;

		var vect = {
			x: this._HDG_HORIZON_POINT.x - base_middle_x,
			y: this._HDG_HORIZON_POINT.y
		};

		var norm_squared = vect.x * vect.x + vect.y * vect.y;
		var factor = Math.sqrt(this._HDG_LENGTH * this._HDG_LENGTH / norm_squared);
		var vect_correct_len = {
			x: vect.x * factor,
			y: vect.y * factor
		};

		var top_x = base_middle_x + vect_correct_len.x;
		var top_left_x = top_x - this._HDG_TOP_HALF_WIDTH;
		var top_right_x = top_x + this._HDG_TOP_HALF_WIDTH;
		var top_y = vect_correct_len.y;

		this._ctx.fillStyle = '#00ff0044';
		this._ctx.beginPath();
		this._ctx.moveTo(base_left_x * this._w, this._h);
		this._ctx.lineTo(base_right_x * this._w, this._h);
		this._ctx.lineTo(top_right_x * this._w, (1 - top_y) * this._h);
		this._ctx.lineTo(top_left_x * this._w, (1 - top_y) * this._h);
		this._ctx.closePath();
		this._ctx.fill();
	}
};

window.addEventListener('load', init_video_funcs, false);
window.addEventListener('load', fetch_fisheye_params, false);
window.addEventListener('load', VideoOverlay._init.bind(VideoOverlay), false);
