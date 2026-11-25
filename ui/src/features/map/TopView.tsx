import { Map as MapIcon } from "lucide-react";

import { PanelCard } from "@/components/PanelCard";
import { LiveImage } from "@/features/camera/LiveImage";

export function TopView() {
  return (
    <PanelCard title="Top View" icon={<MapIcon className="size-4" />}>
      <LiveImage
        name="map"
        alt="Top-down map of the explored area"
        testId="map-image"
        placeholder="The top-down map appears here after the first get_map call."
      />
    </PanelCard>
  );
}
