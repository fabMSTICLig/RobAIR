var webrtc = new SimpleWebRTC({
    // the id/element dom element that will hold "our" video
    localVideoEl: 'localVideo',
    // the id/element dom element that will hold remote videos
    remoteVideosEl: 'remoteVideos',
    // immediately ask for camera access
    autoRequestMedia: true,
    url: config.rtcurl,
    media: {
        audio: true,
        video: {
            width: {
                max: 640
            },
            height: {
                max: 480
            },
            frameRate: {
                max: 15
            },
        }
    }
});
webrtc.on('readyToCall', function() {
    // you can name it anything
    webrtc.joinRoom('robair');
});
