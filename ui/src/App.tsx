import { useEventStream } from "@/api/useEventStream";
import { TooltipProvider } from "@/components/ui/tooltip";
import { PortalLayout } from "@/layout/PortalLayout";

export function App() {
  // One SSE subscription for the lifetime of the app; panels read from the store and query cache.
  useEventStream();
  return (
    <TooltipProvider>
      <PortalLayout />
    </TooltipProvider>
  );
}
