import { useMutation, useQuery, useQueryClient } from "@tanstack/react-query";

import { api } from "@/api/client";
import { isRunStats } from "@/api/sse";
import type { ControlAction, PetState, PetStatus, ProfileName, RunStats, SubmitGoalBody } from "@/types/api";

export const queryKeys = {
  state: ["state"] as const,
  status: ["status"] as const,
  stats: ["stats"] as const,
  settings: ["settings"] as const,
};

/** Retry slowly while the pet server is down; the SSE stream keeps data fresh once it is up. */
const offlineRetryMs = 5_000;

export function useStateQuery() {
  return useQuery({
    queryKey: queryKeys.state,
    queryFn: api.getState,
    staleTime: Number.POSITIVE_INFINITY,
    refetchOnWindowFocus: false,
    retry: false,
    refetchInterval: (query) => (query.state.status === "error" ? offlineRetryMs : false),
  });
}

export function useStatusQuery() {
  return useQuery({
    queryKey: queryKeys.status,
    queryFn: api.getStatus,
    staleTime: Number.POSITIVE_INFINITY,
    refetchOnWindowFocus: false,
    retry: false,
    refetchInterval: (query) => (query.state.status === "error" ? offlineRetryMs : false),
  });
}

/** The run's live counters; the `stats` SSE frames keep this fresh after the first fetch. */
export function useStatsQuery() {
  return useQuery({
    queryKey: queryKeys.stats,
    queryFn: async (): Promise<RunStats | null> => {
      const stats = await api.getStats();
      return isRunStats(stats) ? stats : null;
    },
    staleTime: Number.POSITIVE_INFINITY,
    refetchOnWindowFocus: false,
    retry: false,
    refetchInterval: (query) => (query.state.status === "error" ? offlineRetryMs : false),
  });
}

export function useResetStats() {
  const queryClient = useQueryClient();
  return useMutation({
    mutationFn: () => api.resetStats(),
    onSuccess: (stats: RunStats) => queryClient.setQueryData(queryKeys.stats, stats),
  });
}

export function useSettingsQuery(enabled: boolean) {
  return useQuery({
    queryKey: queryKeys.settings,
    queryFn: api.getSettings,
    enabled,
    staleTime: 60_000,
    retry: false,
  });
}

export function useSubmitGoal() {
  const queryClient = useQueryClient();
  return useMutation({
    mutationFn: (body: SubmitGoalBody) => api.postGoal(body),
    onSuccess: (state: PetState) => queryClient.setQueryData(queryKeys.state, state),
    onSettled: () => queryClient.invalidateQueries({ queryKey: queryKeys.state }),
  });
}

export function useControl() {
  const queryClient = useQueryClient();
  return useMutation({
    mutationFn: (action: ControlAction) => api.control(action),
    onSuccess: (status: PetStatus) => queryClient.setQueryData(queryKeys.status, status),
    onSettled: () => queryClient.invalidateQueries({ queryKey: queryKeys.status }),
  });
}

export function useExtendGoals() {
  const queryClient = useQueryClient();
  return useMutation({
    mutationFn: (goals: number) => api.extendGoals(goals),
    onSuccess: (status: PetStatus) => queryClient.setQueryData(queryKeys.status, status),
    onSettled: () => queryClient.invalidateQueries({ queryKey: queryKeys.status }),
  });
}

export function useSetProfile() {
  const queryClient = useQueryClient();
  return useMutation({
    mutationFn: (name: ProfileName) => api.setProfile(name),
    onSuccess: (status: PetStatus) => queryClient.setQueryData(queryKeys.status, status),
    onSettled: () => {
      queryClient.invalidateQueries({ queryKey: queryKeys.status });
      queryClient.invalidateQueries({ queryKey: queryKeys.state });
    },
  });
}
