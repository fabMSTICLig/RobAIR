var GamepadHandler = {
	_main_gamepad: null,

	loop: function() {
		window.requestAnimationFrame(this.loop.bind(this));

		var pads = navigator.getGamepads();

		for (i in pads) {
			if (!(pads[i] instanceof Gamepad))
				continue;

			if (pads[i].id == this._main_gamepad)
				this.update_speed(pads[i]);

			this.update_buttons(pads[i]);
		}
	},

	update_speed: function(pad) {
		robairros.analogGamepad(pad.axes[0], -pad.axes[1]);
	},

	update_buttons: function(pad) {
	},

	on_connect: function(evt) {
		if (this._main_gamepad == null)
			this._main_gamepad = evt.gamepad.id;
	},

	on_disconnect: function(evt) {
		if (this._main_gamepad._pad.id == evt.gamepad.id) {
			this._main_gamepad = null;

			var pads = navigator.getGamepads();
			for (i in pads) {
				if (pads[i] instanceof Gamepad)
					this._main_gamepad = pads[i].id;
			}
		}
	},

	init: function() {
		var pads = navigator.getGamepads();
		for (i in pads) {
			if (pads[i] instanceof Gamepad)
				this.on_connect({gamepad: pads[i]});
		}

		window.addEventListener('gamepadconnected',
				this.on_connect.bind(this), false);
		window.addEventListener('gamepaddisconnected',
				this.on_disconnect.bind(this), false);
		window.requestAnimationFrame(this.loop.bind(this));
	}
};

GamepadHandler.init();
