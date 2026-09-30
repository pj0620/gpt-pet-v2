import { collapseAllNested, darkStyles, JsonView } from "react-json-view-lite";

import { useSettingsQuery } from "@/api/queries";
import { Separator } from "@/components/ui/separator";
import { Sheet, SheetContent, SheetDescription, SheetHeader, SheetTitle } from "@/components/ui/sheet";
import { Controls } from "@/features/status/Controls";
import { usePortalStore } from "@/stores/portalStore";

interface SettingsSheetProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
}

/** Behind the hamburger: pet controls and the active profile's settings. */
export function SettingsSheet({ open, onOpenChange }: SettingsSheetProps) {
  const connection = usePortalStore((store) => store.connection);
  const settings = useSettingsQuery(open && connection === "open");

  return (
    <Sheet open={open} onOpenChange={(next) => onOpenChange(next)}>
      <SheetContent side="right" className="overflow-y-auto">
        <SheetHeader>
          <SheetTitle>Settings</SheetTitle>
          <SheetDescription>Pet controls and the active profile.</SheetDescription>
        </SheetHeader>
        <div className="flex flex-col gap-4 px-4 pb-4">
          <Controls />
          <Separator />
          <div className="flex flex-col gap-2">
            <h3 className="text-sm font-medium">Profile settings</h3>
            {settings.data ? (
              <div className="rounded-md bg-muted/40 p-2 text-xs">
                <JsonView data={settings.data} style={darkStyles} shouldExpandNode={collapseAllNested} />
              </div>
            ) : (
              <p className="text-xs text-muted-foreground">
                {connection === "open"
                  ? "Loading settings…"
                  : "The pet server is offline; settings appear once it is reachable."}
              </p>
            )}
          </div>
        </div>
      </SheetContent>
    </Sheet>
  );
}
