import { Group, Panel, Separator } from "react-resizable-panels";

import { CameraView } from "@/features/camera/CameraView";
import { EventsLog } from "@/features/events/EventsLog";
import { GoalsQueue } from "@/features/goals/GoalsQueue";
import { TopView } from "@/features/map/TopView";
import { Header } from "@/layout/Header";

const separatorClass =
  "bg-transparent transition-colors hover:bg-ring/40 data-[orientation=horizontal]:h-2 data-[orientation=vertical]:w-2";

/** The mockup's arrangement: goals on the left; camera and map on top right; events below. */
export function PortalLayout() {
  return (
    <div className="flex h-full flex-col bg-background text-foreground">
      <Header />
      <main className="min-h-0 flex-1 px-3 pb-3">
        <Group orientation="horizontal" className="h-full">
          <Panel defaultSize={30} minSize={18}>
            <GoalsQueue />
          </Panel>
          <Separator className={separatorClass} />
          <Panel defaultSize={70} minSize={40}>
            <Group orientation="vertical" className="h-full">
              <Panel defaultSize={62} minSize={30}>
                <Group orientation="horizontal" className="h-full">
                  <Panel defaultSize={55} minSize={25}>
                    <CameraView />
                  </Panel>
                  <Separator className={separatorClass} />
                  <Panel defaultSize={45} minSize={25}>
                    <TopView />
                  </Panel>
                </Group>
              </Panel>
              <Separator className={separatorClass} />
              <Panel defaultSize={38} minSize={18}>
                <EventsLog />
              </Panel>
            </Group>
          </Panel>
        </Group>
      </main>
    </div>
  );
}
