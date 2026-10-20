var Module = {
  canvas: document.getElementById("canvas"),
  onRuntimeInitialized: function () {
    console.log("WASM loaded");

    document.getElementById("pause").addEventListener("change", function () {
      Module._setPause(this.checked);
    });

    document.getElementById("options").addEventListener("change", function () {
      Module._setCorrectionType(Number(this.value));
    });

    document.getElementById("canvas").addEventListener("click", function (e) {
      if (!("ontouchstart" in window || navigator.maxTouchPoints > 0)) {
        // Only on non-touch devices
        Module._setTouchDevice(true);
      }
    });
  },
};
