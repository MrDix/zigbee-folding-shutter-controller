# Hardware

The KiCad design files (schematics, PCB layouts, project libraries) for the two-board stack
will be published here once the first batch of boards has been built and field-tested.

Publishing an unproven layout would invite others to manufacture boards that might still
contain mistakes — the files land here as soon as the design has proven itself in real
windows.

**Overview (final design, pending validation):**

- *Power board* (34 × 42 mm, rounded corners): 24 V input protection (fuse, reverse-polarity
  FET, TVS), two H-bridge motor drivers with current-sense feedback, 24 V → 3.3 V buck
  converter, screw terminals for the 24 V feed and both motors
- *Logic board* (Ø 47 mm): ESP32-C6 module with external antenna connector, input
  conditioning for 8 monitored dry contacts plus wall buttons (via shift registers),
  learn/boot buttons, status LED, programming header
- The boards stack via a 2×10 pin connector and fit a deep flush-mount wall box
