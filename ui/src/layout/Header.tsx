import { Menu } from "lucide-react";
import { useState } from "react";

import { Button } from "@/components/ui/button";
import { StatusBar } from "@/features/status/StatusBar";
import { SettingsSheet } from "@/layout/SettingsSheet";

export function Header() {
  const [settingsOpen, setSettingsOpen] = useState(false);
  return (
    <header className="flex shrink-0 items-center gap-3 px-3 py-2">
      <Button variant="ghost" size="icon" aria-label="Open menu" onClick={() => setSettingsOpen(true)}>
        <Menu />
      </Button>
      <h1 className="font-heading text-base font-medium tracking-tight">GPTPet Management Portal</h1>
      <div className="ml-auto">
        <StatusBar />
      </div>
      <SettingsSheet open={settingsOpen} onOpenChange={setSettingsOpen} />
    </header>
  );
}
