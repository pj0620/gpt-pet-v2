import type { ReactNode } from "react";

import { cn } from "@/lib/utils";

interface PanelCardProps {
  title: string;
  icon?: ReactNode;
  actions?: ReactNode;
  className?: string;
  children: ReactNode;
}

/** A panel of the portal: a card with an `<h2>` heading, optional actions, and a scrollable body. */
export function PanelCard({ title, icon, actions, className, children }: PanelCardProps) {
  return (
    <section
      className={cn(
        "flex h-full min-h-0 flex-col overflow-hidden rounded-xl bg-card text-sm text-card-foreground ring-1 ring-foreground/10",
        className,
      )}
    >
      <header className="flex shrink-0 items-center justify-between gap-2 border-b border-border px-3 py-2">
        <h2 className="flex items-center gap-2 font-heading text-sm font-medium">
          {icon}
          {title}
        </h2>
        {actions ? <div className="flex items-center gap-2">{actions}</div> : null}
      </header>
      <div className="min-h-0 flex-1">{children}</div>
    </section>
  );
}
