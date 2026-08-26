<script>
  import { onMount } from 'svelte';
  import { initRosConnection, rosbridgeUrl } from './lib/ros.js';
  import FlightControlHeader from './lib/components/FlightControlHeader.svelte';
  import DebugImageViewer from './lib/components/DebugImageViewer.svelte';
  import ObservationPanel from './lib/components/ObservationPanel.svelte';
  import ActionPanel from './lib/components/ActionPanel.svelte';
  import TelemetryGauges from './lib/components/TelemetryGauges.svelte';

  onMount(() => {
    // Auto-connect to ROSBridge on page mount
    initRosConnection($rosbridgeUrl);
  });
</script>

<main class="gcs-layout">
  <!-- Top Navigation & Flight Command Bar -->
  <FlightControlHeader />

  <!-- Main Multi-Column Dashboard Grid -->
  <div class="dashboard-grid">
    <!-- Left Column: Video & Actions -->
    <div class="dash-col col-left">
      <!-- Live Perception FPV Viewer -->
      <DebugImageViewer />

      <!-- Policy Action Space (4D) -->
      <ActionPanel />

      <!-- Drone Attitude & Position Telemetry -->
      <TelemetryGauges />
    </div>

    <!-- Right Column: Observation Space (40D) -->
    <div class="dash-col col-right">
      <ObservationPanel />
    </div>
  </div>
</main>

<style>
  .gcs-layout {
    max-width: 1720px;
    margin: 0 auto;
    padding: 16px;
    display: flex;
    flex-direction: column;
    min-height: 100vh;
  }

  .dashboard-grid {
    display: grid;
    grid-template-columns: 1.15fr 0.85fr;
    gap: 16px;
    flex: 1;
  }

  .dash-col {
    display: flex;
    flex-direction: column;
    gap: 16px;
  }

  @media (max-width: 1200px) {
    .dashboard-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
