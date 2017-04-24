var fisheye;
var video_active = false;

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
	}
}

var enable_video_controls = function() {
	$('#fix-distortion')[0].disabled = false;
};

var start_correction = function() {
	update_video();
	$('#undistorted-video')[0].style.display = '';
	$('#remoteVideos')[0].style.display = 'none';
	$('#distortion-value')[0].disabled = false;
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

window.addEventListener('load', init_video_funcs, false);
