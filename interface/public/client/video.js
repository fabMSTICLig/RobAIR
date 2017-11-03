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

window.addEventListener('load', init_video_funcs, false);
window.addEventListener('load', fetch_fisheye_params, false);
